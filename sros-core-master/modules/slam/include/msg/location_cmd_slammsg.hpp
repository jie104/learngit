//
// Created by lfc on 17-9-1.
//

#ifndef SROS_LOCATION_CMD_SLAMMSG_HPP
#define SROS_LOCATION_CMD_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
#include "scan_slammsg.hpp"
#include <Eigen/Dense>

namespace slam{

enum LOCATIONSLAMCMD{
    START_LOCATION_SLAMCMD,    //开始定位
    STOP_LOCATION_SLAMCMD,    //停止定位
    START_LOCAL_LOCATION_SLAMCMD,
    STOP_LOCAL_LOCATION_SLAMCMD,
    START_LOCAL_REALTIME_SLAM_SLAMCMD,
    STOP_LOCAL_REALTIME_SLAM_SLAMCMD,
    START_RELOCATION_SLAMCMD,
};
class LocationCmdSlamMsg: public BaseSlamMsgInterface {
public:
    LocationCmdSlamMsg() : BaseSlamMsgInterface(LOCATION_CMD_SLAMMSG){

    }

    virtual ~LocationCmdSlamMsg(){

    }

    LOCATIONSLAMCMD cmd;

    std::string map_name;
    std::string map_path;
    std::string layer_suffix;
    Eigen::Vector3f pose;
    ScanSlam_Ptr scan;
    bool use_curr_pose = false;
private:



};
typedef std::shared_ptr<LocationCmdSlamMsg> LocationCmd_Ptr;

}


#endif //SROS_LOCATION_CMD_SLAMMSG_HPP
