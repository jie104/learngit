/**
 * @file livox_midxx_module.cpp
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/6/4
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "livox_midxx_module.h"

#include "core/settings.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/point_cloud_msg.hpp"

namespace laser {
// CODE
static uint64_t get_time_in_ns() {
    return static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

LivoxMidXXModule::LivoxMidXXModule()
    : sros::core::Module("Livox_midxx_Driver")
    , is_update_frame_(true)
    , is_enable_publisher_(false) {}

LivoxMidXXModule::~LivoxMidXXModule(){
    is_update_frame_ = false;
};

void LivoxMidXXModule::doOpen() {
    LdsLidar &read_lidar = LdsLidar::GetInstance();
    std::vector<std::string> cmdline_broadcast_code;
    int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code);
    if (!ret) {
        LOG(INFO) << "Init lds lidar success!";
        device_->setStateOK();
    } else {
        device_->setStateOpenFailed();
        LOG(INFO) << "Init lds lidar fail!";
    }
    LOG(INFO) <<"Start discovering device.";
}

bool LivoxMidXXModule::init() {
    sleep(1);  //等待src启动完成
    auto &s = sros::core::Settings::getInstance();
    auto enable_camera_module = (s.getValue<std::string>("device.enable_livox_device", "False") == "True");
    if (!enable_camera_module) {
        LOG(WARNING) << "Livox Module is disabled!";
        return false;
    } else {
        using namespace sros::device;
        device_ = DeviceManager::getInstance()->registerDevice(
            DEVICE_LIDAR_LIVOX,
            DEVICE_ID_LIVOX_MIDXX,
            DEVICE_COMM_INTERFACE_TYPE_ETH_1,
            DEVICE_MOUNT_SROS);
        doOpen();
        // 创建一个新线程,实时读取最新的激光数据并发布
        this->update_frame_thread_ = std::thread(&LivoxMidXXModule::updateFrame, this);
        return true;
    }
}

void LivoxMidXXModule::updateFrame() {
    uint64_t last_frame_time, curr_frame_time;
    last_frame_time = get_time_in_ns();
    LdsLidar &read_lidar = LdsLidar::GetInstance();
    int frame_index = 0;
    LOG(INFO) << "create a new thread for update frame thread";
    while (is_update_frame_) {
        if (kConnectStateSampling != read_lidar.getConnectionState(0)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            LOG(INFO) << "read_lidar.getConnectionState(0)="<<read_lidar.getConnectionState(0);
            continue;
        }

        // 如果获取数据失败,休眠一段时间再尝试获取数据.
        LdsLidar::LivoxExtendRawPointFrame frame;
        if (false == read_lidar.GetFrame(0, frame)) {
            LOG(INFO) << "read_lidar.GetFrame(0, frame)=flase";
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        };

        curr_frame_time = frame.time;
        if (last_frame_time == curr_frame_time) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        // TODO:对外发布数据
        if (is_enable_publisher_) {
            auto msg = std::make_shared<sros::core::PointCloudMsg>("LIVOX_MID70");
            msg->sensor_name = sros::device::DEVICE_LIDAR_LIVOX;
            msg->seq = frame.time;
            msg->time_ = sros::core::util::get_time_in_us();
            msg->cloud.reserve(frame.points.size());
            msg->intensities.reserve(frame.points.size());
            for (const auto &p : frame.points) {
//                msg->cloud.emplace_back(p.x, p.y, p.z);
                msg->cloud.push_back(Eigen::Vector3f(p.x, p.y, p.z));
                msg->intensities.push_back(p.reflectivity);
            }
            last_frame_time = curr_frame_time;
            sendMsg(msg);

//            if (frame_index%100 == 0) {
//                LOG(INFO) << "publish point cloud massage index=" << frame_index;
//            }
            ++frame_index;
        }
    }
    LOG(INFO) << "update frame thread close";
}

void LivoxMidXXModule::run() {
    // 初始化雷达设备
    if (init()) {
        // 接收消息确定是否对外发布激光数据
        subscribeTopic("TOPIC_LIVOX_ENABLE_PUBLISH", CALLBACK(&LivoxMidXXModule::onEnablePublisherMsg));

        // 定时刷新激光状态
        subscribeTopic("TIMER_1S", CALLBACK(&LivoxMidXXModule::onTimer_1s));

        dispatch();
    }
}

void LivoxMidXXModule::onTimer_1s(const sros::core::base_msg_ptr &msg) {
    LdsLidar &read_lidar = LdsLidar::GetInstance();
    if (kConnectStateSampling == read_lidar.getConnectionState(0)) {
        device_->keepAlive();
    } else {
        LOG(INFO) << "livox init state=" << read_lidar.getConnectionState(0);
    }
}

void LivoxMidXXModule::onEnablePublisherMsg(const sros::core::base_msg_ptr &msg) {
    // 根据m->flag设置是否对外发布消息
    auto m = std::dynamic_pointer_cast<sros::core::CommonMsg>(msg);
    if (m && m->flag) {
        LOG(INFO) << "enablePublishLivoxData => true";
        is_enable_publisher_ = true;
    } else {
        LOG(INFO) << "enablePublishLivoxData => false";
        is_enable_publisher_ = false;
    }
}
}