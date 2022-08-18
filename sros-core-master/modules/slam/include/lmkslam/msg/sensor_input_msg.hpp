//
// Created by lfc on 17-6-6.
//

#ifndef LMKSLAM_ALL_SENSOR_MSG_HPP
#define LMKSLAM_ALL_SENSOR_MSG_HPP
#include "base_msg_interface.hpp"
#include "scan_msg.hpp"
#include "pose_msg.hpp"
namespace slam{
class SensorInputMsg : public BaseMsgInterface {
public:
    SensorInputMsg(int64_t time_):BaseMsgInterface(SENSOR_INPUT_MSG) {
        scan.reset(new ScanMsg);
        odo.reset(new PoseMsg);
        stamp = time_;
    }

    virtual ~SensorInputMsg(){

    }

    ScanMsg_Ptr scan;
    PoseMsg_Ptr odo;//后期可能支持三维
private:
    SensorInputMsg():BaseMsgInterface(SENSOR_INPUT_MSG){

    }
};
typedef std::shared_ptr<SensorInputMsg> SensorInputMsg_ptr;
}



#endif //LMKSLAM_ALL_SENSOR_MSG_HPP
