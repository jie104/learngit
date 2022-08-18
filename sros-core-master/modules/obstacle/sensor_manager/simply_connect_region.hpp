//
// Created by lfc on 2021/7/13.
//

#ifndef D435_CALIBRATION_SIMPLY_CONNECT_REGION_HPP
#define D435_CALIBRATION_SIMPLY_CONNECT_REGION_HPP
#include <Eigen/Dense>
#include "bresen_points_builder.hpp"

class SimplyConnectRegion {
 public:
    SimplyConnectRegion(std::vector<Eigen::Vector2f>& points) { buildGrid(points); }

    bool buildGrid(std::vector<Eigen::Vector2f>& points){
        if (points.size()>=3) {
            float min_x, max_x, min_y, max_y;
            min_x = max_x = points[0][0];
            min_y = max_y = points[0][1];
            for (auto& point : points) {
                min_x = min_x < point[0] ? min_x : point[0];
                min_y = min_y < point[1] ? min_y : point[1];
                max_x = max_x > point[0] ? max_x : point[0];
                max_y = max_y > point[1] ? max_y : point[1];
            }
            min_x -= resolution_ * 4;
            max_x += resolution_ * 4;
            min_y -= resolution_ * 4;
            max_y += resolution_ * 4;
            origin_x_ = min_x;
            origin_y_ = min_y;
            length_ = max_x - min_x;
            width_ = max_y - min_y;
            grid_length_ = roundf(length_ / resolution_);
            grid_width_ = roundf(width_ / resolution_);
            if (grid_length_ * grid_width_ < 1024 * 1024) {
                creatGrid(grid_length_, grid_width_);
                updateGrid(points);
            } else {
                LOG(INFO) << "grid is too large! cannot use this Class to creat!" << grid_length_ << "," << grid_width_;
            }
        }
        return false;
    }

    template <class Point>
    bool inRegion(const Point point) const {
        return inRegion(point[0], point[1]);
    }

    bool inRegion(float x, float y) const {
        auto coord_x = coordX(x);
        auto coord_y = coordY(y);
        if (inGrid(coord_x, coord_y)) {
            auto index = coord_y * grid_length_ + coord_x;
            return grid_[index] >= 1;
        }
        return false;
    }

    void getAllPoints(std::vector<Eigen::Vector2i>& points) {
        for (int j = 0; j < grid_width_; ++j) {
            for (int i = 0; i < grid_length_; ++i) {
                if (grid(i, j) >= 1) {
                    points.push_back(Eigen::Vector2i(i, j));
                }
            }
        }
    }

 private:
    void updateGrid(const std::vector<Eigen::Vector2f>& points) {
        tsdf::BresenPointsBuilder<Eigen::Vector2i> bresen_builder;
        std::vector<Eigen::Vector2f> swap_points(points);
        swap_points.push_back(points[0]);
        for (int i = 1; i < swap_points.size(); ++i) {
            Eigen::Vector2i start_i(coordX(swap_points[i - 1][0]), coordY(swap_points[i - 1][1]));
            Eigen::Vector2i end_i(coordX(swap_points[i][0]), coordY(swap_points[i][1]));
            std::vector<Eigen::Vector2i> points_i;
            bresen_builder.computeBresenPoints(start_i, end_i, points_i);
            for (auto& point : points_i) {
                if (inGrid(point[0], point[1])) {
                    grid(point[0], point[1]) = 2;
                }
            }
            if (points_i.size()) {
                buildXcoord(points_i, swap_points[i - 1], swap_points[i]);
            }
        }
        fillRegionByXcoord(x_coords);
        fillCornerLine(points);
    }

    void buildXcoord(std::vector<Eigen::Vector2i>& points_i,const Eigen::Vector2f& start_point,const Eigen::Vector2f& end_point){
        auto start = Eigen::Vector2f(coordXf(start_point[0]), coordYf(start_point[1]));
        auto end = Eigen::Vector2f(coordXf(end_point[0]), coordYf(end_point[1]));
        auto delta_point = end - start;
        auto length_y = (end[1] - start[1]);
        if (points_i.front()[1] != points_i.back()[1]) {
            auto last_point = points_i[0];
            auto delta_y = last_point.cast<float>()[1] - start[1];
            auto x_point = delta_y / length_y * delta_point + start;
            x_coords[last_point[1]].push_back(x_point[0]);
            for (auto& point_local : points_i) {
                if (point_local[1] != last_point[1]) {
                    auto delta_y = point_local.cast<float>()[1] - start[1];
                    auto x_point = delta_y / length_y * delta_point + start;
                    x_coords[point_local[1]].push_back(x_point[0]);
                }
                last_point = point_local;
            }
        }
    }

    void fillRegionByXcoord(const std::vector<std::vector<float>>& x_coords){
        for (int j = 0; j < grid_width_; ++j) {
            for (int i = 0; i < grid_length_; ++i) {
                if (grid(i, j) == 0) {
                    auto& xes = x_coords[j];
                    int left_count = 0, right_count = 0;
                    for (auto& x : xes) {
                        if (x >= i) {
                            left_count++;
                        } else {
                            right_count++;
                        }
                    }
                    if (left_count > 0 && right_count > 0) {
                        //这里为了照应到顶点的情况，顶点需要特殊处理。对于当前的填充，除了部分腰部有多个角点之外的情况都适应。
                        // 为了这个地方可以通过划分三角形来更好的实现。
                        if (left_count % 2 == 1 || right_count % 2 == 1) {
                            grid(i, j) = 1;
                        }
                    }
                }
            }
        }
    }

    void fillCornerLine(const std::vector<Eigen::Vector2f>& points){
        for(auto& point:points){
            int y = coordY(point[1]);
            for (int i = 0; i < grid_length_; ++i) {
                if (grid(i, y) == 0) {
                    if (grid(i, y + 1) > 0 && grid(i, y - 1) > 0) {
                        grid(i, y) = 1;
                    }
                }
            }
        }
    }

    uint8_t& grid(int x, int y) { return grid_[y * grid_length_ + x]; }

    int coordX(const float x) const { return roundf((x - origin_x_) / resolution_); }

    int coordY(const float y) const { return roundf((y - origin_y_) / resolution_); }

    float coordXf(const float x) const { return roundf((x - origin_x_) / resolution_); }

    float coordYf(const float y) const { return roundf((y - origin_y_) / resolution_); }

    bool inGrid(int coord_x, int coord_y) const {
        return coord_x >= 0 && coord_x < grid_length_ && coord_y >= 0 && coord_y < grid_width_;
    }

    void creatGrid(int grid_length, int grid_width) {
        grid_.resize(grid_length * grid_width);
        memset(grid_.data(), 0, grid_length * grid_width * sizeof(uint8_t));
        x_coords.resize(grid_width_);
    }

    const float resolution_ = 0.02f;
    float origin_x_ = 0.0f;
    float origin_y_ = 0.0f;
    float length_ = 0.0f;
    float width_ = 0.0f;
    int grid_length_ = 0;
    int grid_width_ = 0;
    std::vector<uint8_t> grid_;
    std::vector<std::vector<float>> x_coords;
};

#endif  // D435_CALIBRATION_SIMPLY_CONNECT_REGION_HPP
