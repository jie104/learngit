//
// Created by lfc on 17-5-11.
//

#include "slam_module.h"

#include <core/msg/PoseStampedMsg.h>
#include <core/msg/SlamStateMsg.h>
#include <core/settings.h>
#include <core/msg/laser_scan_msg.hpp>
#include "map_processor/map_processor_factory.h"
#include "glog/logging.h"
#include "include/slam_version.h"

using namespace sros::core;

namespace mapping {

SlamModule::SlamModule() : Module("SlamModule") {
    std::string mapping_type_str = sros::core::Settings::getInstance().getValue<std::string>(
            "posefilter.location_type", "SLAM");

    map_processor_type = SLAM_MAP_TYPE;
    if (mapping_type_str == "PGV_IMU") {
        map_processor_type = PGV_MAP_TYPE;
    } else if (mapping_type_str == "SLAM") {
        map_processor_type = SLAM_MAP_TYPE;
    } else if (mapping_type_str == "LMK") {
        map_processor_type = LMK_MAP_TYPE;
    } else {
        LOG(INFO) << "cannot get the type! will use default processor!" << mapping_type_str;
    }
    LOG(INFO) << "the slam mappint type:" << mapping_type_str;
    LOG(INFO) << "slam version:"<<getVersion()<<","<<getVersionStr();

    map_processor = MapProcessorFactory::creatMapProcessor(map_processor_type);
    if (map_processor) {
        map_processor->setSendMsgCallback(boost::bind(&SlamModule::sendMsg, this, _1));
    }else {
        LOG(INFO) << "err to create map processor!";
    }

}

SlamModule::~SlamModule() {

}

void SlamModule::run() {
    LOG(INFO) << "slam module start running";

    subscribeTopic("TOPIC_SLAMCOMMAND", CALLBACK(&SlamModule::syscommandCallback));

    subscribeTopic("TOPIC_LASER", CALLBACK(&SlamModule::scanCallback));

    subscribeTopic("BAG_SCAN", CALLBACK(&SlamModule::scanCallback));

    subscribeTopic("SLAM_PARAMETER", CALLBACK(&SlamModule::paraCallback));

    subscribeTopic("OdoPoseStamped", CALLBACK(&SlamModule::poseCallback));

    usleep(100000);

    if (map_processor) {
        sendStateMsg(map_processor->getState());
    }else {
        LOG(INFO) << "the processor have not creat!";
        sendStateMsg(STATE_SLAM_IDLE);
    }

    dispatch();
}

void SlamModule::syscommandCallback(sros::core::base_msg_ptr base_ptr) {
    if (map_processor) {
        map_processor->processSystemCmd(base_ptr);
    }
}


void SlamModule::sendStateMsg(sros::core::SLAM_STATE_CODE state_msg) {
    sros::core::slam_state_msg_ptr m(new sros::core::SlamStateMsg());
    m->slam_state = state_msg;
    LOG(INFO) << "the msg is:" << state_msg;
    sendMsg(m);
}


void SlamModule::scanCallback(sros::core::base_msg_ptr base_ptr) {
    if (map_processor) {
        map_processor->processScan(base_ptr);
    }
}

void SlamModule::paraCallback(sros::core::base_msg_ptr base_ptr) {
    if (map_processor) {
        map_processor->processPara(base_ptr);
    }
}

void SlamModule::poseCallback(sros::core::base_msg_ptr base_ptr) {
    if (map_processor) {
        map_processor->processPose(base_ptr);
    }
}

int SlamModule::getVersion() {
    return slam::getVersion();
}

std::string SlamModule::getVersionStr() {
    return slam::getVersionStr();
}
}
