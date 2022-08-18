/**
 * @file location_module.cpp
 *
 * @author lhx
 * @date 2016/12/01
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "location_module.h"
#include <core/msg/common_msg.hpp>
#include <iostream>
#include <memory>
#include <string>
#include "core/msg/PoseStampedMsg.h"
#include "core/msg/SlamStateMsg.h"
#include "core/settings.h"
#include "loc_processor/loc_processor_factory.h"
#include "modules/msg_record/debug_lib/base_record_function.h"
#include "modules/msg_record/debug_lib/record_function_operator.hpp"
#include "modules/slam/include/lmkslam/debug_tool/RecordFunc.hpp"

using namespace sros::core;

namespace location {

LocationModule::LocationModule() : Module("LocationModule") {
    std::string location_type_str =
            sros::core::Settings::getInstance().getValue<std::string>("posefilter.location_type", "SLAM");
    LOG(WARNING) << "the location type is:" << location_type_str;
    loc_processor_type = LMK_LOC_TYPE;
    if (location_type_str == "PGV_IMU") {
        loc_processor_type = PGV_LOC_TYPE;
    } else if (location_type_str == "SLAM") {
        loc_processor_type = SLAM_LOC_TYPE;
    } else if (location_type_str == "LMK") {
        loc_processor_type = LMK_LOC_TYPE;
    } else {
        LOG(INFO) << "cannot get the type! will use default processor!" << location_type_str;
    }
    loc_processor = LocProcessorFactory::creatLocProcessor(loc_processor_type);
    loc_processor->setSendMsgCallback(boost::bind(&LocationModule::sendMsg, this, _1));

    //    signal(SIGSEGV, LocationModule::systemExitTrace); //Invaild memory address
    //    signal(SIGABRT, LocationModule::systemExitTrace); // Abort signal
    //    signal(SIGQUIT, LocationModule::systemExitTrace); // Abort signal
    //    signal(SIGINT, LocationModule::systemExitTrace); // Abort signal
    //    signal(SIGKILL, LocationModule::systemExitTrace); // Abort signal
    //    signal(SIGSTOP, LocationModule::systemExitTrace); // Abort signal
}

LocationModule::~LocationModule() {}

void LocationModule::run() {
    LOG(INFO) << "Location module start running";

    subscribeTopic("TOPIC_SLAMCOMMAND", CALLBACK(&LocationModule::syscommandCallback));

    subscribeTopic("TOPIC_LASER", CALLBACK(&LocationModule::scanCallback));

    subscribeTopic("SLAM_PARAMETER", CALLBACK(&LocationModule::paraCallback));

    subscribeTopic("POSEFILTER_PARAMETER", CALLBACK(&LocationModule::paraCallback));

    subscribeTopic("BAG_SCAN", CALLBACK(&LocationModule::scanCallback));

    subscribeTopic("PGV_POSE", CALLBACK(&LocationModule::pgvPoseCallback));

    subscribeTopic("LOCATION_CODE_INFO", CALLBACK(&LocationModule::dmCodeCallback));

    subscribeTopic("TOPIC_ALIGNMENT", CALLBACK(&LocationModule::alignmentCallback));

    subscribeTopic("TIMER_50MS", CALLBACK(&LocationModule::onTime50msLoop));

    subscribeTopic("TOPIC_UPDATE_MAP", CALLBACK(&LocationModule::onUpdateMap));


    usleep(10000);
    if (loc_processor) {
        sendStateMsg(loc_processor->getState());
    } else {
        LOG(INFO) << "the processor have not creat!";
        sendStateMsg(STATE_SLAM_IDLE);
    }
    dispatch();
}

void LocationModule::syscommandCallback(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->processSystemCmd(base_ptr);
    } else {
        LOG(INFO) << "the loc proceesor doesnot exists!";
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
    }
}

void LocationModule::sendStateMsg(sros::core::SLAM_STATE_CODE state_msg) {
    sros::core::slam_state_msg_ptr m(new sros::core::SlamStateMsg());
    m->slam_state = state_msg;
    LOG(INFO) << "the msg is:" << state_msg;
    sendMsg(m);
}

void LocationModule::scanCallback(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->processScan(base_ptr);
    } else {
        LOG(INFO) << "the loc proceesor doesnot exists!";
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
    }
}

void LocationModule::paraCallback(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->processPara(base_ptr);
    } else {
        LOG(INFO) << "the loc proceesor doesnot exists!";
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
    }
}

void LocationModule::systemExitTrace(int signum) {
    const int len = 1024;
    void *func[len];
    size_t size;
    int i;
    char **funs;

    signal(signum, SIG_DFL);
    size = backtrace(func, len);
    funs = backtrace_symbols(func, size);
    LOG(INFO) << "get the backtrace!";
    LOG(INFO) << "the no is:" << signum;
    LOG(INFO) << "System error, Stack trace:";
    auto func_recorder = debug::BaseRecordFunction::getInstance();
    debug::RecordFunctionOperator func_operator("stack trace");
    for (i = 0; i < size; ++i) {
        LOG(INFO) << "the id is:" << i << ",the fun is:" << funs[i];
        func_operator.pushBackFunc(funs[size - 1 - i]);
    }
    debug::RecordFunctionOperator slam_func_operator("slammodule");  // 将slam信息输出出来.
    slam_func_operator.pushBackFunc(debug::RecordFunc::getStr());
    func_recorder->outputAllFuncs();
    free(funs);
    exit(-1);
}

void LocationModule::pgvPoseCallback(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->processPgvPose(base_ptr);
    } else {
        LOG(INFO) << "the loc proceesor doesnot exists!";
        sendStateMsg(sros::core::STATE_SLAM_IDLE);
    }
}

void LocationModule::dmCodeCallback(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->processDmCode(base_ptr);
        auto msg = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(base_ptr);
        sros::core::DataMatrixCodeMsg_ptr send_data_msg(
                        new sros::core::DataMatrixCodeMsg("LOCATION_CODE_ERROR"));
        send_data_msg->loc_x_err_ = msg->loc_x_err_;
        send_data_msg->loc_y_err_ = msg->loc_y_err_;
        send_data_msg->loc_yaw_err_ = msg->loc_yaw_err_;
        sendMsg(send_data_msg);
    }
}
void LocationModule::alignmentCallback(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->processAlignment(base_ptr);
    }
}
void LocationModule::onTime50msLoop(sros::core::base_msg_ptr base_ptr) {
    if (loc_processor) {
        loc_processor->onTime50msLoop();
    }
}

void LocationModule::onUpdateMap(sros::core::base_msg_ptr base_ptr) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(base_ptr);
    const std::string &map_name = msg->str_0_;
    if (map_name == g_state.getCurMapName()) {
        LOG(INFO) << "will update map!";
        if (loc_processor) {
            std::string map_path = MapManager::MAP_SAVE_PATH;
            loc_processor->reloadupdatedMap(map_path,map_name);
        }
    }
}
}  // namespace location
