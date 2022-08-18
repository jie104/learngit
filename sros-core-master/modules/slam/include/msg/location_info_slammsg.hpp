//
// Created by lfc on 17-9-1.
//

#ifndef SROS_LOCATION_INFO_SLAMMSG_HPP
#define SROS_LOCATION_INFO_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
namespace slam{
class LocationInfoSlamMsg: public BaseSlamMsgInterface {
public:
    //TODO:这里有两个部分,一个是slam,一个是posefilter
    LocationInfoSlamMsg():BaseSlamMsgInterface(LOCATION_INFO_SLAMMSG){

    }

    virtual ~LocationInfoSlamMsg(){

    }
    float laser_coordx = 0.285;
    float laser_coordy = 0.0;
    float laser_coordyaw = 0.0;
    float laser_angle_max = 2.37;
    float laser_angle_min = -2.37;
    float laser_z_min = 0.1;
    float laser_z_max = 0.5;
    float laser_max_dist = 30.0;
    float laser_min_dist = 0.1;
    float tilt_angle_thresh = 0.5;
    float map_resolution = 0.02;
    int map_multi_level = 3;
    float pose_percentage_thresh = 0.5;
    int64_t stamp_match_thresh = 60000;

    int ini_angle_count = 40;
    float ini_distri_size = 2.0;
    float consistent_min_err = 0.10;
    int consistent_count_thresh = 3;
    int consistent_pose_size = 20;

    float y_small_err_thresh = 0.03;
    float y_middlet_err_thresh = 0.08;
    float k_y_small_err = 0.4;
    float k_y_middle_err = 0.2;
    float k_y_large_err = 0.1;

    float x_small_err_thresh = 0.05;
    float x_middle_err_thresh = 0.2;
    float k_x_small_err = 1.0;
    float k_x_middle_err = 0.2;
    float k_x_large_err = 0.1;

    float k_theta_small_err = 0.1;
    float k_theta_big_err = 0.6;
    float theta_small_err_thresh = 0.087;


    float pose_dist_err_thresh = 0.08;
    float pose_angle_err_thresh = 0.087;



    float search_space_resolution = 0.02;

    float rt_slam_range_min = 1.0;

    float realtime_update_map_max_dist = 15.0;
    int scan_buffer_size = 100;

    int loc_posert_cache = 100;
    int loc_posemap_cache = 100;

    bool enable_pose_fusion = true;
    bool debug_info_print_flag = false;
    bool use_pitch_roll_correct = false;
    bool use_h_optimize = true;
    bool use_rt_slam_method = true;
    bool use_pose_record = false;
    std::string lmk_type = "CYLINDER";
    bool always_update_pose = false;
    bool realtime_update_map = true;
    int location_layer = 0;
    bool use_scan_to_alignment = false;
    bool use_up_camera_to_alignment = false;
    bool use_down_camera_to_alignment = false;
    bool use_forward_camera_to_alignment = false;
    bool use_left_camera_to_alignment = false;
    bool use_right_camera_to_alignment = false;
    bool use_backward_camera_to_alignment = false;

    // corridor param
    float breakPt_dist_err = 0.05; //过滤两两距离小于breakPt_dist_err的跳变点。
    std::vector<std::vector<float>> corridor_endpoint_feature_value{{0.1,0.53,0.1}};
    std::vector<std::vector<float>> single_beacon_feature_value{{0.1}, {0.05}};
    float beacon_feature_err = 0.03; //信标集合中前后两个跳变点的长度误差。
    float beacon_lenght_err = 0.2;   //信标总长度的误差。
private:


};

typedef std::shared_ptr<LocationInfoSlamMsg> LocationInfo_Ptr;


}



#endif //SROS_LOCATION_INFO_SLAMMSG_HPP
