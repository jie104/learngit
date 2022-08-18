/**
 * @file circle_detect_param.h
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/16
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CIRCLE_DETECT_PARAM_H__
#define __CIRCLE_DETECT_PARAM_H__

// INCLUDE
#include "../base/param_base.hpp"

//CODE
/**
 * @description : TODO
 * @author      : zhangxu
 * @date        : 2020/12/16 下午4:01
 */
struct CircleDetectParam : ParamBase {

    /** @brief min cluster size. */
    int min_cluster_size;

    /** @brief max cluster size. */
    int max_cluster_size;

    /** @brief small values may cause objects to be divided in several clusters. (Unit: m) */
    float point_dist_tolerance;

    /** @brief Image binarization threshold. */
    int threshold;

    /** @brief filter point which normal < normal_angle_tolerance. (Unit: degree) */
    float normal_angle_tolerance;

    /** @brief ransac max iteration.*/
    int max_iteration;

    /** @brief the ratio of fitting points. */
    float fitting_rate;

    /** @brief Maximum distance allowed from pixel to fitting circle. */
    float pixel_dist_allow;

    /** @brief contour min pixel point size. */
    int contour_min_size;

    typedef std::shared_ptr<CircleDetectParam> Ptr;
    typedef std::shared_ptr<const CircleDetectParam> ConstPtr;
};

using CircleDetectParamPtr = CircleDetectParam::Ptr;
using CircleDetectParamConstPtr = CircleDetectParam::ConstPtr;

#endif //PERCEPTION_SOLUTION_CIRCLE_DETECT_PARAM_H
