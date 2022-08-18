//
// Created by lfc on 2020/12/10.
//

#ifndef SROS_LOCATION_CODE_SLAMMSG_HPP
#define SROS_LOCATION_CODE_SLAMMSG_HPP
#include <core/tf/TransForm.h>
#include <Eigen/Dense>
#include <core/msg/data_matrix_code_msg.hpp>
#include "base_slammsg_interface.hpp"

namespace slam{
    
enum FusionTypeChoice {
    ODO_CODE = 0,
    OTHER
}; 
class LocationCodeSlammsg : public BaseSlamMsgInterface{
 public:
    LocationCodeSlammsg():BaseSlamMsgInterface(LOCATION_CODE_SLAMMSG){}
    int64_t matched_scan_time;
    int64_t odo_time;               // 里程计时间戳
    int64_t code_time;
    std::string code_id;
    slam::tf::TransForm code_pose_in_center;
    slam::tf::TransForm delta_pose_to_scan;
    slam::tf::TransForm scan_pose_in_world;
    slam::tf::TransForm code_pose_in_world;
    slam::tf::TransForm realtime_code_pose_in_center;
    Eigen::Vector3f match_world_pose; // 匹配时刻世界坐标系的位姿
    
    int code_type = (int)TYPE_DM_CODE;
    slam::FusionTypeChoice fusion_type = slam::ODO_CODE;  // 融合类型，默认里程计+二维码
    bool is_recorded = false;
    std::string device_name;
};

typedef std::shared_ptr<LocationCodeSlammsg> LocationCodeSlammsg_Ptr;

}

#endif  // SROS_LOCATION_CODE_SLAMMSG_HPP
