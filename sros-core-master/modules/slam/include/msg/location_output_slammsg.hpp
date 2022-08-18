//
// Created by lfc on 17-9-1.
//

#ifndef SROS_LOCATION_OUTPUT_SLAMMSG_HPP
#define SROS_LOCATION_OUTPUT_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
#include "pose_slammsg.hpp"
#include "scan_slammsg.hpp"
#include "core/msg/lmk_match_info_msg.hpp"
namespace slam{

enum LOCATIONSLAMSTATE{
    IDLE_STATE = 1,
    START_LOCATION_SLAMSTATE = 2,
    INITIAL_LOCATION_SLAMSTATE = 3,
    INITIAL_SUCESS_SLAMSTATE = 4 ,
    RELOC_SLAMSTATE = 5,
    SUCCESS_RELOC_SLAMSTATE = 6,
    LOCAL_SLAMSTATE = 7,
    NORMAL_LOCATION_SLAMSTATE = 8,
    WARN_LOCATION_SLAMSTATE = 9,
    ERR_LOCATION_SLAMSTATE = 10,
    FALT_LOCATION_SLAMSTATE = 11,

    BEGGIN_REALTIME_SLAM_SLAMSTATE = 12,
    SUCCESS_BEGGIN_REALTIME_SLAM_SLAMSTATE = 13,
    LOCAL_REALTIME_SLAM_SLAMSTATE = 14,
};
class LocationOutputSlamMsg :public BaseSlamMsgInterface{
public:
    LocationOutputSlamMsg():BaseSlamMsgInterface(LOCATION_OUTPUT_SLAMMSG){

    }

    virtual ~LocationOutputSlamMsg(){

    }

    LOCATIONSLAMSTATE state;
    PoseSlam_Ptr pose;
    ScanSlam_Ptr scan;
    Eigen::Matrix3f cov;
    float score = 0.0f;
    sros::core::LmkMatchinfoMsg_ptr lmk_match_info;
private:


};
typedef std::shared_ptr<LocationOutputSlamMsg> LocationOutput_Ptr;

}


#endif //SROS_LOCATION_OUTPUT_SLAMMSG_HPP
