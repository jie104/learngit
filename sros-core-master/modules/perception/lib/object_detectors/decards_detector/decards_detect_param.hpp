//
// Created by ljh on 2021/12/2.
//

#ifndef PERCEPTION_SOLUTION_DESTACK_DETECT_PARAM_HPP
#define PERCEPTION_SOLUTION_DESTACK_DETECT_PARAM_HPP
// INCLUDE
#include "../base/param_base.hpp"

// CODE
/**
 * @brief detect parameters
 * @author      : lijunhong
 * @date        : 2021/12/2 上午9:47
 */
struct DecardsDetectParam : ParamBase
{

    /** @brief Image binarization threshold. */
    int threshold{};

    /** @brief */
    int upper_lip_height{};

    int stack_top_margin{};

    int stack_min_area{};

    int stack_max_area{};

    int stack_min_col{};

    int stack_max_col{};

    float stack_min_rectangularity{};

    float stack_max_rectangularity{};

    int target_points_min_count{};

    int target_points_max_count{};

    float target_min_width_radio{};

    float target_max_width_radio{};

    /** @brief min cluster size. */
    int min_cluster_size{};

    /** @brief max cluster size. */
    int max_cluster_size{};

    /** @brief small values may cause objects to be divided in several clusters. (Unit: m) */
    float point_dist_tolerance{};

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
    
    typedef  std::shared_ptr<DecardsDetectParam> Ptr;
    typedef  std::shared_ptr<const DecardsDetectParam> ConstPtr;
};

using DecardsDetectParamPtr = DecardsDetectParam::Ptr;
using DecardsDetectParamConstPtr = DecardsDetectParam::ConstPtr;



#endif //PERCEPTION_SOLUTION_DESTACK_DETECT_PARAM_HPP
