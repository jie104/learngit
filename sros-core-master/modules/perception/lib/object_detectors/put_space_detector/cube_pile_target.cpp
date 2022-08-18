/**
 * @file target.cpp
 * @brief target class function implementation
 * 
 * target class function implementation. such as get/set function.
 * 
 * @author lijunhong@standard-robots.com
 * @date create date：2022/1/15
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "cube_pile_target.h"

namespace perception {
perception::CubePileTarget::CubePileTarget()
    : Target() {

}

perception::CubePileTarget::~CubePileTarget() {

}

void
perception::CubePileTarget::reset() {

    Target::reset();
    front_side_.clear();
    top_side_.clear();
    this->id_ = -1;

}

} // end of namespace perception