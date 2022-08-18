/**
 * @file target.cpp
 * @brief target class function implementation
 * 
 * target class function implementation. such as get/set function.
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/9/22
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "handcart_target.h"

namespace perception {
perception::HandcartTarget::HandcartTarget()
    : Target()
    , width_(0)
    , height_(0) {

}

perception::HandcartTarget::~HandcartTarget() {

}

void
perception::HandcartTarget::reset() {

    Target::reset();

    this->width_ = 0;
    this->height_ = 0;
    this->left_down_.x = 0;
    this->left_down_.y = 0;
    this->right_down_.x = 0;
    this->right_down_.y = 0;
    this->id_ = -1;

    this->rect_.x = 0;
    this->rect_.y = 0;
    this->rect_.width = 0;
    this->rect_.height = 0;
}

} // end of namespace perception