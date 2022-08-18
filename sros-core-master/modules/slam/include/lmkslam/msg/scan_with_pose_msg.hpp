//
// Created by lfc on 17-8-1.
//

#ifndef SROS_SCAN_WITH_POSE_HPP
#define SROS_SCAN_WITH_POSE_HPP

#include "base_msg_interface.hpp"
#include "scan_msg.hpp"
#include "pose_msg.hpp"
namespace slam{
class ScanWithPoseMsg: public BaseMsgInterface{
public:
    ScanWithPoseMsg():BaseMsgInterface(SCAN_WITH_POSE_MSG){

    }
    ScanMsg_Ptr scan;
    PoseMsg_Ptr pose;
private:


};

}



#endif //SROS_SCAN_WITH_POSE_HPP
