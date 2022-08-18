//
// Created by lfc on 17-7-7.
//

#ifndef SROS_LOCATION_PROCESSOR_H
#define SROS_LOCATION_PROCESSOR_H

#include <glog/logging.h>
#include <core/msg/SlamStateMsg.h>
#include "core/msg/SlamCommandMsg.h"
#include <memory>
#include <boost/function.hpp>
//#include <bits/signum.h>
#include <signal.h>
#include <execinfo.h>

namespace location{
enum LOCATION_STATE {
    NORMAL_LOC_STAT = 1,
    STOP_LOC_STAT = 2,
    ERR_LOC_STAT = 3,
    RELOC_LOC_STAT = 4,
};
enum LOC_PROCESSOR_TYPE {
    PGV_LOC_TYPE = 1,
    SLAM_LOC_TYPE = 2,
    LMK_LOC_TYPE = 4,
};
typedef boost::function<void(sros::core::base_msg_ptr)> SendMsgCallback;
class LocationProcessor {
public:
    LocationProcessor(LOC_PROCESSOR_TYPE type): processor_type(type) {
        location_system_state = sros::core::STATE_SLAM_IDLE;
    }

    void setSendMsgCallback(SendMsgCallback callback) {
        sendMsg = callback;
    }

    virtual bool processScan(sros::core::base_msg_ptr base_ptr) {
        return true;
    }

    virtual bool processScan(sros::core::base_msg_ptr base_msg_ptr, sros::core::Pose base_pose) {
        return true;
    }

    virtual bool processPara(sros::core::base_msg_ptr base_ptr) {
        return true;
    }

    virtual void processPgvPose(sros::core::base_msg_ptr base_ptr){

    }

    void processSystemCmd(sros::core::base_msg_ptr base_ptr){
        auto syscommand_msg = std::dynamic_pointer_cast<sros::core::SlamCommandMsg>(base_ptr);

        sros::core::SlamCommandMsg &syscommand = *syscommand_msg;

        using namespace sros::core;
        switch (location_system_state) {
            case STATE_SLAM_IDLE:
                switch (syscommand.slam_command) {
                    case COMMAND_START_LOCATION_MANUAL:
                        changeStateMsg(STATE_SLAM_WAITING_INITIAL_POSE);
                        handleStartLocation(syscommand);
                        break;
                    case COMMAND_START_LOCATION_AUTO:

                        break;

                    case COMMAND_START_DRAW_MAP:

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
                        handleStopLocation(syscommand);
                        break;

                    case COMMAND_START_LOCAL_REAL_TIME_SLAM_LOCATION:
                        handleStartLocalRealTimeSlamLoc(syscommand);
                        break;

                    case COMMAND_STOP_LOCAL_REAL_TIME_SLAM_LOCATION:
                        handleStopLocalRealTimeSlamLoc(syscommand);
                        break;

                    case COMMAND_START_LOCAL_LOCATION:
                        LOG(INFO) << "start locallocation!";
                        handleStartLocalLocation(syscommand);
                        break;
                    case COMMAND_STOP_LOCAL_LOCATION:
                        LOG(INFO) << "stop locallocation";
                        handleStopLocalLocation(syscommand);
                        break;
                    case COMMAND_START_RELOCATION:
                        LOG(INFO) << "will relocate curr pose!";
                        changeStateMsg(STATE_SLAM_WAITING_INITIAL_POSE);
                        handleRelocation(syscommand);
                        break;
                    default:
                        LOG(INFO) << "err cmd!" << syscommand.slam_command;
                        break;
                }
                break;
            case STATE_SLAM_DRAWING:
                switch (syscommand.slam_command) {
                    case COMMAND_STOP_DRAW_MAP:

                        break;
                    case COMMAND_CANCEL_DRAW_MAP:

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

    bool handleLocState(location::LOCATION_STATE state) {
        using namespace sros::core;
        switch (location_system_state) {
            case STATE_SLAM_IDLE: {
                if (state == RELOC_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_LOCATING_AMCL);
                }else {
                    LOG(INFO) << "the state cannot convert! will return false" << state;
                    return false;
                }

                break;
            }
            case STATE_SLAM_LOCATING: {
                if (state == RELOC_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_LOCATING_AMCL);
                }else if (state == ERR_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_ERROR);
                } else if (state == STOP_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_IDLE);
                }else{
                    LOG(INFO) << "the state cannot convert! will return false" << state;
                    return false;
                }
                break;
            }
            case STATE_SLAM_ERROR: {
                if (state == STOP_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_IDLE);
                } else{
                    LOG(INFO) << "the state cannot convert! will return false" << state;
                    return false;
                }
                break;
            }
            case STATE_SLAM_LOCATING_AMCL: {
                if (state == NORMAL_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_LOCATING);
                } else if (state == ERR_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_ERROR);
                }else if (state == STOP_LOC_STAT) {
                    return changeStateMsg(STATE_SLAM_IDLE);
                }else {
                    LOG(INFO) << "the state cannot convert! will return false" << state;
                    return false;
                }
                break;
            }
        }
        LOG(INFO) << "the state cannot convert! will return false" << state;
        return false;
    }

    virtual bool handleStartLocation(sros::core::SlamCommandMsg &syscommand) {
        //有些需要重新定位,故而,不能在基类中实现函数,而应在子类中实现.下面,将实现默认的状态变换
        handleLocState(RELOC_LOC_STAT);
        usleep(10000);
        return handleLocState(NORMAL_LOC_STAT);
    }

    virtual bool handleRelocation(sros::core::SlamCommandMsg &syscommand) {
        //有些需要重新定位,故而,不能在基类中实现函数,而应在子类中实现.下面,将实现默认的状态变换
        handleLocState(RELOC_LOC_STAT);
        usleep(10000);
        return handleLocState(NORMAL_LOC_STAT);
    }

    virtual bool handleStopLocation(sros::core::SlamCommandMsg &syscommand) {
        return handleLocState(STOP_LOC_STAT);

    }
    virtual bool handleStartLocalRealTimeSlamLoc(sros::core::SlamCommandMsg &syscommand){
        return false;
    }

    virtual bool handleStopLocalRealTimeSlamLoc(sros::core::SlamCommandMsg &syscommand){
        return false;
    }
    virtual bool handleStartLocalLocation(sros::core::SlamCommandMsg &syscommand){
        return false;
    }

    virtual bool handleStopLocalLocation(sros::core::SlamCommandMsg &syscommand){
        return false;
    }

    virtual void processDmCode(sros::core::base_msg_ptr base_ptr) {

    }

    virtual void processAlignment(sros::core::base_msg_ptr base_ptr) {

    }

    virtual bool loadMap(std::string map_name) {
        return false;
    }

    virtual void onTime50msLoop(){

    }

    sros::core::SLAM_STATE_CODE getState() {
        return location_system_state;
    }

    LOC_PROCESSOR_TYPE getType() {
        return processor_type;
    }

    virtual void reloadupdatedMap(const std::string map_path,const std::string map_name){

    }
protected:
    sros::core::SLAM_STATE_CODE location_system_state;
    SendMsgCallback sendMsg;

    bool changeStateMsg(sros::core::SLAM_STATE_CODE state) {
        location_system_state = state;
        sendStateMsg(state);
        return true;
    }

private:
    void sendStateMsg(sros::core::SLAM_STATE_CODE state_msg){
        sros::core::slam_state_msg_ptr m(new sros::core::SlamStateMsg());
        m->slam_state = state_msg;
        LOG(INFO) << "the msg is:" << state_msg;
        if(sendMsg){
            sendMsg(m);
        }
    }
    LOC_PROCESSOR_TYPE processor_type;
    LocationProcessor(){

    }
};

typedef std::shared_ptr<LocationProcessor> LocationProcessor_Ptr;

}


#endif //SROS_LOCATION_PROCESSOR_H
