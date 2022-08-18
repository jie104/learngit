//
// Created by lfc on 17-7-10.
//

#ifndef SROS_MAPPING_PROCESSOR_H
#define SROS_MAPPING_PROCESSOR_H

#include <glog/logging.h>
#include <core/msg/SlamStateMsg.h>
#include "core/msg/SlamCommandMsg.h"
#include <memory>
#include <boost/function.hpp>
#include <core/msg/laser_scan_msg.hpp>
#include <cmath>

namespace mapping{
typedef boost::function<void(sros::core::base_msg_ptr)> SendMsgCallback;
enum MAP_PROCESSOR_TYPE{
    SLAM_MAP_TYPE = 1,
    LMK_MAP_TYPE = 3,
    PGV_MAP_TYPE = 4,
};
class MappingProcessor {
public:
    MappingProcessor(MAP_PROCESSOR_TYPE type): processor_type(type) {
            mapping_system_state = sros::core::STATE_SLAM_IDLE;
    }

    void processSystemCmd(sros::core::base_msg_ptr base_ptr){
        auto syscommand_msg = std::dynamic_pointer_cast<sros::core::SlamCommandMsg>(base_ptr);

        sros::core::SlamCommandMsg &syscommand = *syscommand_msg;

        using namespace sros::core;
        LOG(INFO) << "the state is" << mapping_system_state;
        LOG(INFO) << "the cmd is:" << syscommand.slam_command;
        switch (mapping_system_state) {
            case STATE_SLAM_IDLE:
                switch (syscommand.slam_command) {
                    case COMMAND_START_LOCATION_MANUAL:

                        break;
                    case COMMAND_START_LOCATION_AUTO:

                        break;

                    case COMMAND_START_DRAW_MAP:
                        handleStartMapping(syscommand);
                        break;

                    default:
                        break;
                }
                break;
            case STATE_SLAM_WAITING_INITIAL_POSE:
                break;
            case STATE_SLAM_SIFT_LOCATING:
                break;
            case STATE_SLAM_LOCATING:
                switch (syscommand.slam_command) {
                    case COMMAND_STOP_LOCATION:

                        break;
                    default:
                        break;
                }
                break;
            case STATE_SLAM_DRAWING:
                switch (syscommand.slam_command) {
                    case COMMAND_STOP_DRAW_MAP:
                        handleStopMapping(syscommand);
                        break;
                    case COMMAND_CANCEL_DRAW_MAP:
                        handleCancelMapping(syscommand);
                        break;
                    default:
                        break;
                }
                break;
            case STATE_SLAM_SAVING_MAP:
                break;
            case STATE_SLAM_LOCATING_AMCL:
                break;
            case STATE_SLAM_ERROR:
                break;
            default:
                break;

        }
        if (syscommand.slam_command == sros::core::COMMAND_LOAD_MAP |
            syscommand.slam_command == sros::core::COMMAND_INIT_COMPLETE) {
            //TODO:需要到serial驱动里设置
        }
    }

    virtual bool handleStartMapping(sros::core::SlamCommandMsg &syscommand) {
        //有些需要重新定位,故而,不能在基类中实现函数,而应在子类中实现.下面,将实现默认的状态变换
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
        return false;
    }

    virtual bool handleStopMapping(sros::core::SlamCommandMsg &syscommand) {
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
        return false;
    }

    virtual bool handleCancelMapping(sros::core::SlamCommandMsg &syscommand) {
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
        return false;
    }

    void setSendMsgCallback(SendMsgCallback callback) {
        sendMsg = callback;
    }

    virtual bool processScan(sros::core::base_msg_ptr base_ptr) {
        return true;
    }

    virtual bool processPose(sros::core::base_msg_ptr base_ptr) {
        return true;
    }

    virtual bool processPara(sros::core::base_msg_ptr base_ptr) {
        return true;
    }

    virtual void saveMap(std::string map_name){

    }

    sros::core::SLAM_STATE_CODE getState() {
        return mapping_system_state;
    }



    MAP_PROCESSOR_TYPE getType() {
        return processor_type;
    }
protected:


    bool changeStateMsg(sros::core::SLAM_STATE_CODE state,sros::core::Progress_t progress = 0) {

        mapping_system_state = state;
        sendStateMsg(state,progress);
        return true;
    }

    virtual bool checkScanState(sros::core::base_msg_ptr base_ptr){
        sros::core::LaserScan_ptr scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
        if (!scan) {
            LOG(INFO) << "cannot get the scan! will return false";
            return false;
        }
        int com_point_size = floorf((scan->angle_max - scan->angle_min) / scan->angle_increment + 0.5 + 1.0);
        if(com_point_size == scan->ranges.size()) {
            return true;
        }else {
            LOG(INFO) << "the scan size is wrong!" << scan->ranges.size();
            return false;
        }
    }

    sros::core::SLAM_STATE_CODE mapping_system_state;

    SendMsgCallback sendMsg;

private:

    void sendStateMsg(sros::core::SLAM_STATE_CODE state_msg, sros::core::Progress_t progress = 0){
        sros::core::slam_state_msg_ptr m(new sros::core::SlamStateMsg());
        m->slam_state = state_msg;
        m->progress = progress;
//        LOG(INFO) << "the msg is:" << state_msg;
        if(sendMsg){
            sendMsg(m);
        }
    }

    MAP_PROCESSOR_TYPE processor_type;

};

typedef std::shared_ptr<MappingProcessor> MappingProcessor_Ptr;
}



#endif //SROS_MAPPING_PROCESSOR_H
