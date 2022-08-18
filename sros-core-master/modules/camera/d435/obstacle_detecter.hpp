#pragma once

#include <librealsense2/rs.hpp>
#include "../stereo_point.h"
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include "obstacle_grid.hpp"
#include "math.hpp"
#include "point_image.hpp"
#include "serialization.hpp"
#include <Eigen/Dense>

namespace obstacle_detection {
class ObstacleDetecter {
public:
    ObstacleDetecter() : point_image_(height_, width_) {}

private:
    static constexpr int height_ = 240;                   //图像高度(z方向)
    static constexpr int width_ = 424;                    //图像宽度(y方向)
    static constexpr int low_bound_ = 4;                  //寻找前后点的下界
    static constexpr int up_bound_ = 40;                  //寻找前后点的上界
    static constexpr float max_curvature_ = 1.f;          //最大斜率
    static constexpr float min_delta_x_ = 0.02f;          //最大x变化值
    static constexpr float max_delta_x_ = 0.2f;           //最大x变化值
    static constexpr float max_z_ = 0.15f;                //最大z值
    static constexpr float z_bias_ = 0.1f;                //z值偏差
    static constexpr float valid_x_ = 4.f;
    static constexpr float valid_y_ = 0.6f;

    PointImage point_image_;                              //栅格化点图像
    StereoPoints point_cloud_;                            //点云
    ObstacleGrid obstacle_grid_;                          //障碍物栅格

    Eigen::Matrix3f calibration_matrix_;                  //校准矩阵
    float install_height_ = 0.122f;
    float min_height_ = 0.03f;                            //最小高度
    std::tuple<float, float, float, float> rack_parameters_;
public:
    //处理
    void process(const rs2::points &points) {
        getPoints(points);
        getFrontAndBackIndex();
        getScore();
        getObstacleGrid();
    }

    const ObstacleGrid &obstacleGrid() const { return obstacle_grid_; }

    const StereoPoints &pointCloud() const { return point_cloud_; }


    bool calibrateCameraInstallErr(const rs2::points &points, const double install_theta,
                                   const double calibration_height, double &install_height,
                                   Eigen::Matrix3d &calibration_matrix) {
        LOG(INFO) << install_theta << " "<< calibration_height<< " " << install_height;
        const float sin_theta = std::sin(install_theta); //sin(θ)，预计算优化效率
        const float cos_theta = std::cos(install_theta); //cos(θ)，预计算优化效率
        const auto &vertices = points.get_vertices();
        const auto &texture_coordinates = points.get_texture_coordinates();
        const auto stream_profile = points.get_profile().as<rs2::video_stream_profile>();
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        Eigen::Matrix3f squared_mean = Eigen::Matrix3f::Zero();
        unsigned int size = 0;
        for (int row = 0; row < height_; row++) {
            for (int col = 0; col < width_; col++) {
                const int i = row * width_ + col;
                const auto &vertice = vertices[i];
                if (!std::isfinite(vertice.x) || !std::isfinite(vertice.y) || !std::isfinite(vertice.z)) {
                    continue;
                } else if (math::isZero(vertice.x) && math::isZero(vertice.y) && math::isZero(vertice.z)) {
                    continue;
                } else {
                    Eigen::Vector3f point(vertice.z, -vertice.x, -vertice.y);
                    if (std::abs(
                            point.z() * cos_theta - point.x() * sin_theta + install_height + calibration_height) <
                        z_bias_ &&
                        point.z() * sin_theta + point.x() * cos_theta < valid_x_ &&
                        std::abs(point.y()) < valid_y_) {
                        ++size;
                        mean += point;
                        squared_mean += point * point.transpose();
                    }
                }
            }
        }
        if (size < 100)
            return false;
        const float size_inv = 1.f / static_cast<float>(size);
        mean *= size_inv;
        squared_mean *= size_inv;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(squared_mean - mean * mean.transpose());
        Eigen::Vector3f norm = eigen_solver.eigenvectors().col(0);
        if (norm.z() < 0) {
            norm = -norm;
        }
        install_height = -norm.dot(mean) - calibration_height;
        Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(norm, Eigen::Vector3f::UnitZ());
        calibration_matrix  = quat.matrix().cast<double>();
        return true;
    }

    void updateCameraInstallErr(const Eigen::Matrix3d &calibration_matrix, const double install_height) {
        calibration_matrix_ = calibration_matrix.cast<float>();
        install_height_ = install_height;
    }

    void setMinHeight(const float min_height) {
        min_height_ = min_height;
    }

private:
    //获得点集
    void getPoints(const rs2::points &points) {
        const auto &vertices = points.get_vertices();
        const auto &texture_coordinates = points.get_texture_coordinates();
        const auto stream_profile = points.get_profile().as<rs2::video_stream_profile>();
        for (int row = 0; row < height_; row++) {
            for (int col = 0; col < width_; col++) {
                const int i = row * width_ + col;
                const auto &vertice = vertices[i];
                auto &point = point_image_(row, col);
                if (!std::isfinite(vertice.x) || !std::isfinite(vertice.y) || !std::isfinite(vertice.z)) {
                    point.validity = false;
                } else if (math::isZero(vertice.x) && math::isZero(vertice.y) && math::isZero(vertice.z)) {
                    point.validity = false;
                } else {
                    point.validity = true;
                    Eigen::Vector3f p(vertice.z, -vertice.x, -vertice.y);
                    p = calibration_matrix_ * p;
                    point.x = p.x();
                    point.y = p.y();
                    point.z = p.z();
                    if (point.z < -install_height_) {
                        const float k = -point.z / install_height_;
                        point.x *= k;
                        point.y *= k;
                        point.z = -install_height_;
                    }
                    point.intensity = 0;
                }
            }
        }
    }

    //获得前后点序数
    void getFrontAndBackIndex() {
        for (int row = 0; row < height_; row++) {
            for (int col = 0; col < width_; col++) {
                auto &point = point_image_(row, col);
                if (point.validity) {
                    int valid_pos = 0;
                    for (int pos = low_bound_; pos < up_bound_ && row - pos >= 0; pos++) {
                        if (point_image_(row - pos, col).validity) {
                            valid_pos = pos;
                            if (std::abs(point_image_(row - pos, col).x - point.x) >=
                                min_delta_x_) {
                                break;
                            }
                        }
                    }
                    point.front_index = row - valid_pos;
                    valid_pos = 0;
                    for (int pos = low_bound_; pos < up_bound_ && row + pos < height_;
                         pos++) {
                        if (point_image_(row + pos, col).validity) {
                            valid_pos = pos;
                            if (std::abs(point_image_(row + pos, col).x - point.x) >=
                                min_delta_x_) {
                                break;
                            }
                        }
                    }
                    point.back_index = row + valid_pos;
                }
            }
        }
    }

    //获得各点分数
    void getScore() {
        point_cloud_.clear();
        point_cloud_.reserve(width_ * height_);
        StereoPoints::Point point_xyzi;
        for (int row = 0; row < height_; row++) {
            for (int col = 0; col < width_; col++) {
                auto &p = point_image_(row, col);
                if (p.validity && p.front_index != row && p.back_index != row) {
                    float score = 0.f;
                    const auto &pf = point_image_(p.front_index, col);
                    const auto &pb = point_image_(p.back_index, col);
                    const float delta_bz = std::abs(pb.z - p.z);
                    const float delta_bx = std::abs(pb.x - p.x);
                    if (delta_bx > min_delta_x_ || delta_bz > min_delta_x_) {
                        float curvature = 0.f;
                        const float alpha_f =
                                std::atan(std::abs(pf.z - p.z) / std::abs(pf.x - p.x));
                        const float alpha_b =
                                std::atan(delta_bz / delta_bx);
                        const float z_mean = (pf.z + pb.z) * 0.5f;
                        if (p.z < z_mean) {
                            curvature = std::min(std::abs(alpha_f), std::abs(alpha_b));
                        } else {
                            curvature = std::abs(alpha_f - alpha_b);
                        }
                        score = std::min(1.f, curvature / max_curvature_);
                    }
                    point_xyzi.x = p.x;
                    point_xyzi.y = p.y;
                    point_xyzi.z = p.z;
                    point_xyzi.inten = score;
                    point_cloud_.push_back(point_xyzi);
                }
            }
        }
    }

    //获得障碍物栅格
    void getObstacleGrid() {
        obstacle_grid_.reset();
        auto &points = point_cloud_.points;
        for (const auto &point : points) {
            if (point.z > max_z_)
                continue;
            if (point.z < -install_height_ + min_height_)
                continue;
            obstacle_grid_.set(point.x, point.y, point.inten);
        }
        obstacle_grid_.open();
        obstacle_grid_.close();
        obstacle_grid_.thresh();
    }
};
}// namespace obstacle_detection