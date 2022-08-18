//
// Created by lfc on 17-9-1.
//

#ifndef SROS_SLAM_INFO_MSG_HPP
#define SROS_SLAM_INFO_MSG_HPP
#include "base_slammsg_interface.hpp"
namespace slam{
class MappingInfoSlamMsg: public BaseSlamMsgInterface {
public:
    MappingInfoSlamMsg() : BaseSlamMsgInterface(MAPPING_INFO_SLAMMSG){

    }

    virtual ~MappingInfoSlamMsg(){

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
    int64_t stamp_match_thresh = 60000;

    bool use_pitchroll_correct = 1;
    bool use_kslam = true;
    bool use_lmkslam = true;
    int lmkscan_size_thresh = 10;
    bool use_feature_correct = true;
    float loop_search_max_dist = 3.0;
    float lmk_intensity_thresh = 500;
    float length_double_lmk_thresh = 0.34;
    bool use_recorded_bag = 0;
    std::string lmk_type = "CYLINDER";
    bool need_endstart_loop = true;
};

typedef std::shared_ptr<MappingInfoSlamMsg> MappingInfo_Ptr;
}



#endif //SROS_SLAM_INFO_MSG_HPP
