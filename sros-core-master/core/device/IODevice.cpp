/**
 * @file IODevice.cpp
 *
 * @author pengjiali
 * @date 2019年5月20日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "IODevice.h"
#include <glog/logging.h>
#include <functional>
#include <thread>
#include "core/util/utils.h"

namespace sros {
namespace device {

IODevice::IODevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                   std::shared_ptr<IOInterface> io_interface)
    : Device(name, device_id, device_comm_interface_type, DEVICE_MOUNT_SROS), io_interface_ptr_(io_interface) {
    io_interface_ptr_->setReadCallback(std::bind(&IODevice::dataReciveCallback, this, std::placeholders::_1));
}

void IODevice::init(const std::vector<uint8_t> &data) {
    auto func = [&] { request_ = data; };

    for (int i = 0; i < 3; ++i) {
        setRequest(func);
        if (syncRequest()) {
            setStateOK();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    setStateInitialFailed();
}

void IODevice::dataReciveCallback(const std::vector<uint8_t> &data) {
    keepAlive();
    onDataRecive(data);
}

void IODevice::setRequest(std::function<void()> fun) {
    request_.clear();
    response_.clear();
    is_wating_for_response_ = false;
    is_error_ = false;

    fun();
    is_wating_for_response_ = true;
}

bool IODevice::syncRequest() {
    async_condition_.reset();
    io_interface_ptr_->write(request_);
    bool is_response_ok = false;  // 记录回复是否正确
    bool ret = async_condition_.waitForResult(1000, is_response_ok);
    if (!ret) {
        LOG(INFO) << getName() << " response超时！ request: " << numberListToStr(request_.cbegin(), request_.cend());
        last_error_code_ = ERROR_CODE_RESPONSE_TIMEOUT;
        return false;
    }

    if (!is_response_ok) {
        LOG(INFO) << getName() << " response 错误！ " << numberListToStr(request_.cbegin(), request_.cend());
        last_error_code_ = ERROR_CODE_RESPONSE_INORRECT;
        return false;
    }
    return true;
}

bool IODevice::asyncRequest(const std::vector<uint8_t> &data) { return io_interface_ptr_->write(data); }

void IODevice::setResponse(bool ok) { async_condition_.setResult(ok); }
}  // namespace device
}  // namespace sros
