//
// Created by lfc on 18-12-13.
//



#ifndef SROS_OBSTACLE_MODULE_PARA_HPP
#define SROS_OBSTACLE_MODULE_PARA_HPP

#include "core/pose.h"
#include <memory>
#include <Eigen/Dense>

namespace oba {
struct SH100Info{
    bool sh100_enable_state = false;
    int sh100_id = 0x310;
};

template <class T=float>
struct Range3D{
    /** @brief pass through filter point.x > min_x. (Unit: m) */
    T min_x;

    /** @brief pass through filter point.x < max_x. (Unit: m) */
    T max_x;

    /** @brief pass through filter point.y > min_y. (Unit: m) */
    T min_y;

    /** @brief pass through filter point.y < max_y. (Unit: m) */
    T max_y;

    /** @brief pass through filter point.z > min_z. (Unit: m) */
    T min_z;

    /** @brief pass through filter point.z < max_z. (Unit: m) */
    T max_z;
};

struct ObstacleModulePara {
    double laser_coord_x = 0.29;
    double laser_coord_y = 0.0;
    double laser_coord_yaw = 0.0;
    double laser_angle_min = -2.1;
    double laser_angle_max = 2.1;
    double oba_laser_range_min = 0.05;
    double oba_laser_range_max = 30.0;
    double r2100_coord_x = -0.3;
    double r2100_coord_y = 0;
    double r2100_coord_yaw = 0;
    double stereo_camera_coord_x = 0.39;
    double stereo_camera_coord_y = 0.0;
    double stereo_camera_coord_yaw = 0.0;
    double stereo_camera_2_coord_x = 0.39;
    double stereo_camera_2_coord_y = 0.0;
    double stereo_camera_2_coord_yaw = 0.0;
    double stereo_camera_3_coord_x = 0.39;
    double stereo_camera_3_coord_y = 0.0;
    double stereo_camera_3_coord_yaw = 0.0;
    double stereo_camera_4_coord_x = 0.39;
    double stereo_camera_4_coord_y = 0.0;
    double stereo_camera_4_coord_yaw = 0.0;

    // IFM Install offset 
    double ifm_camera_coord_x = 0.0;
    double ifm_camera_coord_y = 0.0;
    double ifm_camera_coord_yaw = 0.0;

    // IFM obstacle detect range 
    Range3D<float> detect_range{};
    sros::core::Pose target_position;


    double near_tof_dist_with_high_strength = 0.5;
    uint16_t near_tof_dist_strength = 500;
    const int sh100_nums = 3;
    std::vector<SH100Info> sh100_infos = std::vector<SH100Info>(sh100_nums);
    const int tof_nums = 12;
    int each_sh100_tof_num = 4;
    std::vector<Eigen::Vector3d> tof_coords = std::vector<Eigen::Vector3d>(tof_nums, Eigen::Vector3d::Zero());
    std::vector<bool> sh100_tof_enable_states = std::vector<bool>(tof_nums,true);
    std::vector<bool> online_sh100_tof_enable_states = std::vector<bool>(tof_nums,true);
    std::vector<bool> soft_touch_tof_states = std::vector<bool>(tof_nums,false);
    bool enable_sh100_debug_output = false;

    bool enable_r2000_obs_online = true;
    bool enable_ust_forward_online = true;
    bool enable_ust_back_online = true;
    bool enable_ust_left_online = true;
    bool enable_ust_right_online = true;
    bool enable_ust_forward_device = true;
    bool enable_ust_back_device = true;
    bool enable_ust_left_device = true;
    bool enable_ust_right_device = true;
    double ust_detect_min_height = 0.05;
    double ust_detect_max_height = 1.0;

    int eu100_tim320_stop_state = 0;
    bool eu100_tim320_enable = true;
    bool use_stereo_points = true;
    bool use_ifm_points = true;
    bool is_load_full_state = false;
    bool enable_remove_rack_leg = true;
    bool enable_filter_only_load_full = true;
    bool enable_r2100_points_when_load_full = false;
    bool enable_filter_low_intensity_points = false;
    double min_filter_length = 1.2f;
    double rack_leg_center_length = 1.2;
    double rack_leg_center_width = 0.6;
    double rack_leg_diameter = 0.1;
    double rack_backlash_rotate_angle = 0.034;
    double rack_radius_offset = 0.034;
    double rack_direction = 0.0;
    float car_heght = 10.0;

    bool enable_switch_region_avoid_obstacle = false;
    bool enable_left_tim320 = true;
    bool enable_right_tim320 = true;
    bool enable_back_tim320 = false;
    bool enable_photoelectric_switch_avoid_obstacle = true;
    int left_tim320_id = 0x331;
    int right_tim320_id = 0x332;
    int back_tim320_id = 0x333;

    int64_t delta_time_thresh = 60000;  //获取当前数据时间戳相近的位姿,该值为时间戳阈值,微秒
};

typedef std::shared_ptr<ObstacleModulePara> ObstacleModulePara_Ptr;
};

#endif //SROS_OBSTACLE_MODULE_PARA_HPP
