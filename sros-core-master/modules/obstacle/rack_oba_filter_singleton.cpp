//
// Created by lfc on 19-8-12.
//

#include "rack_oba_filter_singleton.h"

namespace oba {
std::shared_ptr<RackObaFilterSingleton> RackObaFilterSingleton::location_3d_module_;
std::mutex RackObaFilterSingleton::thread_mutex_;

std::shared_ptr<RackObaFilterSingleton> RackObaFilterSingleton::getInstance() {
    if (!location_3d_module_) {
        std::lock_guard<std::mutex> lock_guard(thread_mutex_);//在抛出异常时,依然不会发生死锁现象.
        if (!location_3d_module_) {
            location_3d_module_.reset(new RackObaFilterSingleton());
        }
    }
    return location_3d_module_;
}

RackObaFilterSingleton::RackObaFilterSingleton() {

}

bool RackObaFilterSingleton::isRackPoint(const Eigen::Vector2d &point) {
    for (auto &rectangle:rectangle_regions_) {
        if (rectangle.inRectangle(point)) {
            return true;
        }
    }
    return false;
}

void RackObaFilterSingleton::updateRackPara(const ObstacleModulePara_Ptr &para) {
    int num_times = (int) round(para->rack_backlash_rotate_angle / rectangle_angle_step);

    double max_x = (para->rack_leg_center_length / 2.0 + para->rack_leg_diameter);
    double min_x = -max_x;
    double max_y = (para->rack_leg_center_width / 2.0 + para->rack_leg_diameter);
    double min_y = -max_y;
    rack_rectangle_.reset(new Rectangle);
    rack_rectangle_->max_x = max_x;
    rack_rectangle_->max_y = max_y;
    rack_rectangle_->min_x = min_x;
    rack_rectangle_->min_y = min_y;

    rectangle_regions_.clear();
    for (int i = -num_times; i <= num_times; ++i) {
        rectangle_regions_.emplace_back();
        rectangle_regions_.back().updateInfo(rack_rectangle_, double(i) * rectangle_angle_step);
    }
}

void RackObaFilterSingleton::updateRackRectangleInfo(const ObstacleModulePara_Ptr &para) {
    if (!rack_rectangle_) {
        rack_rectangle_.reset(new Rectangle);
    }
    double max_x = (para->rack_leg_center_length / 2.0 + para->rack_leg_diameter);
    double min_x = -max_x;
    double max_y = (para->rack_leg_center_width / 2.0 + para->rack_leg_diameter);
    double min_y = -max_y;
    rack_rectangle_->max_x = max_x;
    rack_rectangle_->max_y = max_y;
    rack_rectangle_->min_x = min_x;
    rack_rectangle_->min_y = min_y;
}

void RackObaFilterSingleton::computeRackDirection(double rack_direction) {
    for (auto &rectangle:rectangle_regions_) {
        rectangle.computeTFInfo(rack_direction);
    }
}

}