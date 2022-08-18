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
#include "card_target.h"

namespace perception {
perception::CardTarget::CardTarget()
    : Target()
    , card_width_(0)
    , hole_height_(0) {

}

perception::CardTarget::~CardTarget() {

}

void
perception::CardTarget::reset() {

    Target::reset();

    this->card_width_ = 0;
    this->hole_height_ = 0;
    this->left_down_.x = 0;
    this->left_down_.y = 0;
    this->right_down_.x = 0;
    this->right_down_.y = 0;
    this->id_ = -1;

    this->rect_.x = 0;
    this->rect_.y = 0;
    this->rect_.width = 0;
    this->rect_.height = 0;

    this->hole_rect_.clear();
    this->pallet_rect_.clear();
}

void
perception::CardTarget::setPalletsIndices(
    const int start_index,
    const int num) {

    assert(start_index >= 0 && num >= 0);
    this->pallet_indices_.clear();
    for (int i = start_index; i < start_index + num; i++) {
        this->pallet_indices_.push_back(i);
    }
}

void
perception::CardTarget::setPalletRect(const std::vector<cv::Rect> &pallets) {

    this->pallet_rect_.assign(pallets.begin(), pallets.end());

    // get left-up point(min_x, min_y) and right-down point(max_x, max_y).
    int min_x, max_x, min_y, max_y;
    min_x = INT_MAX, max_x = INT_MIN;
    min_y = INT_MAX, max_y = INT_MIN;
    for (auto rect: this->pallet_rect_) {
        min_x = MIN(rect.x, min_x);
        min_y = MIN(rect.y, min_y);
        max_x = MAX(rect.x + rect.width, max_x);
        max_y = MAX(rect.y + rect.height, max_y);
        //LOG(INFO) << "("<< rect.x << ", "<< rect.y << ")-->("<< rect.x + rect.width << ", " << rect.y + rect.height << ")" ;
    }

    //LOG(INFO) << "("<< min_x << ", "<< min_y << ")-->("<< max_x << ", " << max_y << ")" ;
    this->rect_.x = min_x;
    this->rect_.y = min_y;
    this->rect_.width = max_x - min_x + 1;
    this->rect_.height = max_y - min_y + 1;
}

} // end of namespace perception