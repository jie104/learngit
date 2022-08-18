/**
 * @file circle_target.cpp
 * @brief Circular target class
 *
 * Target class for detecting circular targets
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/16
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "circle_target.h"

// CODE
namespace perception {

void CircleTarget::reset() {
    //Target::reset();

    this->id_ = -1;
    this->angle_ = 180;
    this->pose_.x = 0;
    this->pose_.y = 0;
    this->pose_.z = 0;

    this->pose_normal_.normal_x = 0;
    this->pose_normal_.normal_y = 0;
    this->pose_normal_.normal_z = 0;

    this->normal_cloud_->clear();
    this->point_cloud_->clear();

    this->score_ = 0;
    this->radius_ = 0;
    this->radius_pixel_ = 0;
    this->contour_img_.setTo(0);
    this->contour_points_.clear();
    this->center_in_image_.x = 0;
    this->center_in_cloud_.y = 0;
    this->center_in_cloud_.reset();
}

} // end of namespace perception