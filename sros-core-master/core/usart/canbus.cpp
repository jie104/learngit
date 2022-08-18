//
// Created by john on 19-5-20.
//

#include "canbus.h"

#include <thread>
#include <stdexcept>
#include <boost/thread.hpp>
#include "core/settings.h"
#include "core/util/utils.h"

Canbus::Canbus() {
    auto &s = sros::core::Settings::getInstance();
    device_name_ = s.getValue<std::string>("hmi.can_device_name", "vcan0");

    auto enable_can_bus = (s.getValue<std::string>("hmi.enable_can_bus", "True") == "True");

    if (!enable_can_bus) {
        LOG(ERROR) << "Can module disabled";
        return;
    }

    state_ = STATE::RUNNING;
    std::thread t(&Canbus::runCan, this);
    t.detach();
}

Canbus &Canbus::getInstance() {
    static Canbus sigleton;
    return sigleton;
}

void Canbus::runCan() {
    LOG(INFO) << "thread Canbus::runCan() started";

    while (state_ == STATE::RUNNING) {

        /* CAN device for music player and hmi touch */
        can_ = std::make_shared<SocketCan>();

        if (!can_->open(device_name_)) {
            LOG(WARNING) << "socket can open failed, retry after 500ms...";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        LOG(INFO) << "socket can opened!";

        can_->setMessageCallbackFunc(
                std::bind(&Canbus::onCanMsgCallback, this, std::placeholders::_1, std::placeholders::_2));

        can_->startRecv();

        // 如果startRecv函数返回，说明socket出现了错误，那么需要重新创建socket

        std::unique_lock<std::mutex> unique_lock(mutex_can_);
        can_.reset();
        unique_lock.unlock();

        LOG(INFO) << "runCan loop break, reopen can socket";

        boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
    }

    LOG(INFO) << "thread Canbus::runCan() exited";
}

void Canbus::addDevice(canid_t response_id, sros::device::IOInterfaceReadDataFunc func) {
    std::lock_guard<std::mutex> lock_guard(mutex_can_);
    read_callback_map_[response_id] = func;
}

void Canbus::sendData(canid_t can_id, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock_guard(mutex_can_);
    if (!can_) {
        return;
    }

    bool ret = can_->send(can_id, data);
    if (!ret) {
        LOG(WARNING) << "Canbus: can send data failed! can_id: 0x" << std::hex << can_id;
    }
}

void Canbus::onCanMsgCallback(canid_t can_id, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock_guard(mutex_can_);
    if (read_callback_map_.find(can_id) != read_callback_map_.end()) {
        read_callback_map_.at(can_id)(data);
    }
}
