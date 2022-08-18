//
// Created by lfc on 19-8-12.
//

#ifndef SROS_RACK_OBA_FILTER_SINGLETON_H
#define SROS_RACK_OBA_FILTER_SINGLETON_H

#include <memory>
#include <mutex>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "obstacle_module_para.hpp"

namespace oba {
class Rectangle {
public:
    Rectangle() {

    }

    virtual bool inRectangle(const double &coord_x, const double &coord_y) const {
        if (coord_x >= min_x && coord_x <= max_x) {
            if (coord_y >= min_y && coord_y <= max_y) {
                return true;
            }
        }
        return false;
    }

    double min_y = 0.6 / 2;
    double max_y = -0.6 / 2;
    double min_x = -1.06 / 2.0;
    double max_x = 1.06 / 2.0;
};

typedef std::shared_ptr<Rectangle> Rectangle_Ptr;

class RectangleInfo {
public:
    void updateInfo(Rectangle_Ptr rectangle, double direction) {
        rectangle_ = rectangle;
        direction_offset_ = direction;
        cos_theta = cos(direction_offset_);
        sin_theta = sin(direction_offset_);
    }

    void computeTFInfo(double direction) {
        cos_theta = cos(direction_offset_ + direction);
        sin_theta = sin(direction_offset_ + direction);
    }

    bool inRectangle(const Eigen::Vector2d &point) {
        double tmp_x = point[0] * cos_theta + point[1] * sin_theta;
        double tmp_y = point[1] * cos_theta - point[0] * sin_theta;
        return rectangle_->inRectangle(tmp_x, tmp_y);
    }

private:
    Rectangle_Ptr rectangle_;
    double direction_offset_ = 0.0;
    double cos_theta;
    double sin_theta;
};

class RackObaFilterSingleton {
public:
    static std::shared_ptr<RackObaFilterSingleton> getInstance();

    bool isRackPoint(const Eigen::Vector2d &point);

    void updateRackPara(const ObstacleModulePara_Ptr &para);

    void computeRackDirection(double rack_direction);

    void updateRackRectangleInfo(const ObstacleModulePara_Ptr &para);

private:
    RackObaFilterSingleton();

    static std::shared_ptr<RackObaFilterSingleton> location_3d_module_;
    static std::mutex thread_mutex_;
    Rectangle_Ptr rack_rectangle_;
    std::vector<RectangleInfo> rectangle_regions_;
    double rectangle_angle_step = 0.034;

};


}


#endif //SROS_RACK_OBA_FILTER_SINGLETON_H
