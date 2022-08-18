//
// Created by lfc on 2021/4/25.
//

#ifndef LIVOX_GROUND_PLANE_MAP_HPP
#define LIVOX_GROUND_PLANE_MAP_HPP
#include <vector>

namespace standard {
template <class Point>
class GroundPlaneMap {
 public:
    struct HeightBar {
        void addPoint(int index, const Point& point){
            if (points_index.empty()) {
                max_height = point.z();
                min_height = point.z();
                points.emplace_back(Eigen::Vector3f(0,0,0));
                weights.emplace_back(0.0f);//给定初始值，该值不在系统中使用
            }
            if (points_index.size() <= index) {
                int points_size = points_index.size();
                points_index.reserve(index + 1);
                for (int i = points_size; i < index + 1; ++i) {
                    points_index.push_back(-1);//所有小格子默认值为洞（-1），只有填充时，值大于等于1
                }
            }
            if (points_index[index] != -1) {//已经被填充过的小格子，进行均值滤波
                auto& array_index = points_index[index];
                points[array_index] =
                    (points[array_index] * weights[array_index] + point) * (1.0 / (weights[array_index] + 1));
                weights[array_index] += 1.0f;
            }else{//没有被填充的，进行索引，注意该索引是从1开始计数
                points_index[index] = points.size();
                points.push_back(point);
                weights.push_back(1.0);
            }
            max_height = max_height > point.z() ? max_height : point.z();//不管地面处于什么位置，条的高度是通过相对高度计算的，max-min
            min_height = min_height < point.z() ? min_height : point.z();
        }

        const Eigen::Vector3f pointByIndex(int index) const { return points[points_index[index]]; }

        bool heightEnough(float height){ return max_height - min_height > height; }

        std::vector<int16_t> points_index;
        std::vector<Point> points;
        std::vector<float> weights;
        float max_height;
        float min_height;
    };

    GroundPlaneMap(float resolution, float length, float width, float min_height, float max_height) //75
        : resolution_(resolution), min_height_(min_height), max_height_(max_height), height_step_(75.0f) {
        length_ = roundf(length / resolution);
        width_ = roundf(width / resolution);
        map_.resize(length_ * width_);
        center_x_offset_ = 0.0;
        center_y_offset_ = width / 2.0;
//        clear();
    }

    const float resolution(){ return resolution_; }

    void clear() {
        map_.clear();
        map_.resize(length_ * width_);
    }

    bool inHeight(const float height) const { return height > min_height_ && height < max_height_; }

    bool inGrid(const int x, const int y) const { return x >= 0 && x < length_ && y >= 0 && y < width_; }

    int heightStepCount(uint16_t state) {
        int countx = 0;
        while (state) {
            countx++;
            state = state & (state - 1);
        }
        return countx;
    }

    int coordX(const float coord_x) const { return (int)roundf((coord_x + center_x_offset_) / resolution_); }

    float toWorldX(const int coord_x)const{ return coord_x * resolution_ - center_x_offset_; }

    float toWorldY(const int coord_y)const{ return coord_y * resolution_ - center_y_offset_; }

    int coordY(const float coord_y) const { return (int)roundf((coord_y + center_y_offset_) / resolution_); }

    const HeightBar operator()(const float coord_x, const float coord_y) const {
        int coord_x_int = coordX(coord_x);
        int coord_y_int = coordY(coord_y);
        if (inGrid(coord_x_int, coord_y_int)) {
            return map_[coord_y_int * length_ + coord_x_int];
        }
        return HeightBar();
    }

    void addPoint(float coord_x, float coord_y,const Point& point) {
        int coord_x_int = (int)roundf((coord_x + center_x_offset_) / resolution_);//沿着X方向不需要膨胀
        int coord_y_int = (int)floorf((coord_y + center_y_offset_) / resolution_);//沿着Y方向需要膨胀一格，消除点云过希的影响
        int coord_h_int = (int)floorf((point.z() - min_height_) * height_step_);//沿着深度方向需要膨胀一格，消除高度方向过希的影响
        if (inGrid(coord_x_int, coord_y_int) && inGrid(coord_x_int + 1, coord_y_int + 1) && coord_h_int >= 0) {
            for (int i = 0; i < 1; ++i) {
                for (int j = 0; j < 2; ++j) {
                    for (int k = 0; k < 2; ++k) {
                        bar(coord_x_int + i, coord_y_int + j).addPoint(coord_h_int + k, point);//填充柱条
                    }
                }
            }
        }
    }

    HeightBar& bar(int x, int y) { return map_[y * length_ + x]; }

    HeightBar& barByWorld(const float coord_x, const float coord_y) {
        return map_[coordY(coord_y) * length_ + coordX(coord_x)];
    }

    int length(){ return length_; }

    int width(){ return width_; }

    float toHeight(int index){ return float(index) / height_step_ + min_height_; }

    int barIndex(float height) const { return roundf((height - min_height_) * height_step_); }

    const float heightStep()const { return height_step_; }

 private:
    std::vector<HeightBar> map_;
    int length_, width_;
    float center_x_offset_;
    float center_y_offset_;
    float resolution_;
    float min_height_;
    float max_height_;
    float height_step_;
};
}  // namespace livox

#endif  // D435_CALIBRATION_GROUND_PLANE_MAP_HPP
