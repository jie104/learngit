//
// Created by lfc on 2021/4/15.
//

#ifndef D435_CALIBRATION_GRIDPLANEBUILDER_HPP
#define D435_CALIBRATION_GRIDPLANEBUILDER_HPP
#include <memory>
#include <vector>

//class PointImage {
// public:
//    struct PointElement {
//        Eigen::Vector3f point;
//        uint16_t coord_x;
//        uint16_t coord_y;
//        bool valid = false;
//    };
//
//    PointImage(uint16_t length, uint16_t width) : length_(length), width_(width) { points_.resize(length * width); }
//
//    PointElement &operator()(const uint16_t &coord_x, const uint16_t &coord_y) {
//        return points_[coord_y * length_ + coord_x];
//    }
//
//    const PointElement &operator()(const uint16_t &coord_x, const uint16_t &coord_y) const {
//        return points_[coord_y * length_ + coord_x];
//    }
//
//    const uint16_t &length() const { return length_; }
//
//    const uint16_t &width() const { return width_; }
//
// private:
//    std::vector<PointElement> points_;
//    uint16_t length_;
//    uint16_t width_;
//};
//
//struct PlaneInfo{
//    Eigen::Vector3f norm;
//    Eigen::Vector3f mean;
//    float weight;
//    uint16_t coord_x;
//    uint16_t coord_y;
//};

class GridPlaneBuilder {
 public:
    template<class PointImage,class PlaneInfo>
    static void computeLocalPlanes(const PointImage& image,std::vector<std::shared_ptr<PlaneInfo>>& planes){
        const int sample_step = 3;
        auto length = image.length() - sample_step;
        auto width = image.width() - sample_step;
        for (int i = sample_step; i < width; i += sample_step) {
            for (int j = sample_step; j < length; j += sample_step) {
                std::shared_ptr<PlaneInfo> plane(new PlaneInfo);
                if (computePlane(image, j, i, sample_step, plane)) {
                    planes.push_back(plane);
                }
            }
        }
    }

    //目前处理的时候，图片存储的坐标信息已经转换成了右手系，如果没有转换的，可能需要做一些修改。
    template <class PointImage,class PlaneInfo>
    static bool computePlane(const PointImage& image,const uint16_t coord_x,const uint16_t coord_y,const int step,std::shared_ptr<PlaneInfo> &plane){
        auto &point = image(coord_x, coord_y);
        if (point.valid) {
            auto &mean = point.point;
            double rotate_angle = 0;
            const double angle_step = M_PI_4;
            const double angle_rotate_offset = M_PI_2;
            float initial_coord = step;
            plane->coord_x = coord_x;
            plane->coord_y = coord_y;
            plane->mean = mean;
            plane->norm.setZero();
            for (int i = 0; i < 8; ++i) {
                auto x = roundf(initial_coord * cos(rotate_angle) + coord_x);
                auto y = roundf(initial_coord * sin(rotate_angle) + coord_y);

                auto rotate_x = roundf(initial_coord * cos(rotate_angle + angle_rotate_offset) + coord_x);
                auto rotate_y = roundf(initial_coord * sin(rotate_angle + angle_rotate_offset) + coord_y);
                auto& first_point = image(x, y);
                auto& second_point = image(rotate_x, rotate_y);
                if (first_point.valid && second_point.valid) {
                    Eigen::Vector3f first_vector = first_point.point - mean;
                    Eigen::Vector3f second_vector = second_point.point - mean;
                    auto norm = first_vector.cross(second_vector);
                    norm.normalize();
                    plane->norm += norm;
                }
                rotate_angle += angle_step;
            }
            plane->weight = plane->norm.norm();
            if (plane->weight >= 5.0) {
                plane->norm.normalize();
                plane->norm *= -1.0f;//考虑到图像坐标是左手系，而实际的点云为右手系，乘以-1即可转换成右手系的向量。
                return true;
            }
            return false;
        }
        return false;
    }

};

#endif  // D435_CALIBRATION_GRIDPLANEBUILDER_HPP
