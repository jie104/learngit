/**
 * @file card_detect_param.hpp
 * @brief detect parameters
 *
 * Some parameters used in point cloud data processing,  For example, in order to speed
 * up the search and restrict the search range, the range parameter(min_x,max_x, min_y,
 * max_y, min_z, max_z) is added. For details, please refer to the parameter notes.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CARD_DETECT_PARAM_HPP__
#define __CARD_DETECT_PARAM_HPP__
//INCLUDE
#include "../base/param_base.hpp"

// CODE
/**
 * @brief detect parameters
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:15
 */
struct CardDetectParam : ParamBase
{
    /** @brief min cluster size. */
    int min_cluster_size{};

    /** @brief max cluster size. */
    int max_cluster_size{};

    /** @brief small values may cause objects to be divided in several clusters. (Unit: m) */
    float point_dist_tolerance{};

    /** @brief Image binarization threshold. */
    int threshold{};

    /** @brief filter point which normal < normal_angle_tolerance. (Unit: degree) */
    float normal_angle_tolerance{};

    /** @brief allow distance between both pallet. (Unit: m) */
    float both_pallet_dist_allow{};

    /** @brief search for extended pixels in the up direction. (Unit: pixel) */
    int up_extend_pixel{};

    /** @brief search for extended pixels in the down direction. (Unit: pixel) */
    int down_extend_pixel{};

    /** @brief hole min_area = area * min_area_rate, if hole_area < max_area then drop. */
    float min_area_rate{};

    /** @brief hole max_area = area * max_area_rate, if hole_area > max_area then drop. */
    float max_area_rate{};

    /** @brief hole rate = width/height,  if hole rate > hole_height_rate_diff_allow the drop. */
    float hole_height_rate_diff_allow{};

    /** @brief between two normal angle different allowed. */
    float pallet_normal_diff_angle_allow{};

    /** @brief enable record debug image*/
    bool enable_record_debug_image{};

    /** @brief filter point which normal < normal_ground angle_tolerance. (Unit: degree) */
    float normal_groud_angle_tolerance{};

    // // 目标栈板的高度
    // float target_value_z_ = 0.;
    // //相机沿Z轴放到地面时的安装偏移量
    // Eigen::Vector3f camera_to_agv_transf;
    // //相机沿Z轴放到地面时的旋转偏移
    // Eigen::Vector3f camera_to_agv_rotate;  // pitch yaw, roll

    typedef  std::shared_ptr<CardDetectParam> Ptr;
    typedef  std::shared_ptr<const CardDetectParam> ConstPtr;
};

using CardDetectParamPtr = CardDetectParam::Ptr;
using CardDetectParamConstPtr = CardDetectParam::ConstPtr;

#endif //PERCEPTION_SOLUTION_CARD_DETECT_PARAM_HPP
