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

#ifndef __HANDCART_DETECT_PARAM_HPP__
#define __HANDCART_DETECT_PARAM_HPP__
//INCLUDE
#include "../base/param_base.hpp"

// CODE
/**
 * @brief detect parameters
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:15
 */
struct HandcartDetectParam : ParamBase
{
    /** @brief min cluster size. */
    int min_cluster_size{};

    /** @brief max cluster size. */
    int max_cluster_size{};

    /** @brief small values may cause objects to be divided in several clusters. (Unit: m) */
    float point_dist_tolerance{};

    /** @brief Image binarization threshold. */
    int rate_threshold{};

    /** @brief filter point which normal < normal_angle_tolerance. (Unit: degree) */
    float normal_angle_tolerance{};

    /** @brief Allowable range of across deviation between real length and theoretical length. (Unit: m) */
    float across_dist_allow{};

    typedef  std::shared_ptr<HandcartDetectParam> Ptr;
    typedef  std::shared_ptr<const HandcartDetectParam> ConstPtr;
};

using HandcartDetectParamPtr = HandcartDetectParam::Ptr;
using HandcartDetectParamConstPtr = HandcartDetectParam::ConstPtr;

#endif //PERCEPTION_SOLUTION_CARD_DETECT_PARAM_HPP
