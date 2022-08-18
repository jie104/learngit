/**
 * @file device.cpp
 *
 * @author lhx
 * @date 2018年1月31日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "device.h"
#include <glog/logging.h>

#include "core/logger.h"
#include "core/fault_center.h"
#include "core/state.h"

namespace sros {
namespace device {

Device::Device(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
               DeviceMountType device_type)
    : name_(name),
      device_id_(device_id),
      state_(DEVICE_NONE),
      serial_no_("-"),
      model_no_("-"),
      version_no_("-"),
      time_(0),
      comm_interface_type_(device_comm_interface_type),
      mount_type_(device_type) {
    setStateInitialization();
}

void Device::setStateError(uint32_t raw_fault_code) {
    if (state_ != DEVICE_ERROR) {
        state_ = DEVICE_ERROR;
        LOGGER(INFO, DEVICE) << "Device " << name_ << " error! raw fault code is " << raw_fault_code;
    }

    if (raw_fault_code != raw_fault_code_) {
        raw_fault_code_ = raw_fault_code;
        auto old_unicorn_fault_code_ = unicorn_fault_code_;
        unicorn_fault_code_ = faultCodeMapping(raw_fault_code) + device_id_ * 1000;

        if (raw_fault_code == 0) {
            LOGGER(INFO, DEVICE) << "Device " << getName() << " fault clean: " << old_unicorn_fault_code_ << " -> "
                                 << unicorn_fault_code_;
        } else {
            LOGGER(ERROR, DEVICE) << "Device " << getName() << " set raw fault code 0x" << std::hex << raw_fault_code;
            LOGGER(ERROR, DEVICE) << "Device " << getName() << " fault change: " << old_unicorn_fault_code_ << " -> "
                                  << unicorn_fault_code_;

            auto fault_center = sros::core::FaultCenter::getInstance();
            fault_center->removeFault(old_unicorn_fault_code_);
            fault_center->addFault(unicorn_fault_code_);
        }
    }
}

void Device::keepAlive() {
    updateAliveTime();

    if (state_ == DEVICE_ERROR_TIMEOUT) {  // 只有在超时的状态，才允许设置为ok
        setStateOK();
    }
}

void Device::checkAlive() {
    if (state_ != DEVICE_OK) {
        return;
    }

    if (time_ == 0) {
        updateAliveTime();
    }

    auto cur_time = sros::core::util::get_time_in_ms();

    if (cur_time > timeout_time_ + time_) {
        setStateTimeout();
    }
}

void Device::setStateNone() {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_NONE) {
        state_ = DEVICE_NONE;
        LOGGER(INFO, DEVICE) << "Device " << name_ << " is DEVICE_NONE!";
    }
}

void Device::setStateInitialization() {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_INITIALIZING) {
        state_ = DEVICE_INITIALIZING;
        LOGGER(INFO, DEVICE) << "Device " << name_ << " in initializing!";
    }
}

void Device::setStateOK() {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_OK) {
        state_ = DEVICE_OK;
        cleanFault();
        updateSrosEmergencySrc(false);
        LOGGER(INFO, DEVICE) << "Device " << name_ << " communication resumes normal!";
    }
}

void Device::setStateOff() {
    if (state_ != DEVICE_OFF) {
        state_ = DEVICE_OFF;
        cleanFault();
        LOGGER(INFO, DEVICE) << "Device " << name_ << " normal closed!";
    }
}

void Device::setStateOpenFailed() {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_ERROR_OPEN_FAILED) {
        state_ = DEVICE_ERROR_OPEN_FAILED;
        raw_fault_code_ = 0;
        uint32_t old_unicorn_fault_code = unicorn_fault_code_;
        unicorn_fault_code_ = DEVICE_FAULT_OPEN + device_id_ * 1000;
        setUnicornFaultCode(old_unicorn_fault_code,unicorn_fault_code_);
        LOGGER(ERROR, DEVICE) << "Device " << name_ << " open failed, set fault code " << unicorn_fault_code_;
    }
}

// 需初始化设备连接状态
void Device::setStateOpening() {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_OPENING) {
        state_ = DEVICE_OPENING;
        raw_fault_code_ = 0;
        uint32_t old_unicorn_fault_code = unicorn_fault_code_;
        unicorn_fault_code_ = DEVICE_FAULT_OPENING + device_id_ * 1000;
        setUnicornFaultCode(old_unicorn_fault_code,unicorn_fault_code_);
        LOGGER(ERROR, DEVICE) << "Device " << name_ << " is opening, set fault code " << unicorn_fault_code_;
    }
}

void Device::setStateInitialFailed() {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_ERROR_INITIAL) {
        state_ = DEVICE_ERROR_INITIAL;
        raw_fault_code_ = 0;
        uint32_t old_unicorn_fault_code = unicorn_fault_code_;
        unicorn_fault_code_ = DEVICE_FAULT_INITIAL + device_id_ * 1000;
        setUnicornFaultCode(old_unicorn_fault_code,unicorn_fault_code_);
        LOGGER(ERROR, DEVICE) << "Device " << name_ << " initial failed, set fault code " << unicorn_fault_code_;
    }
}
void Device::setStateTimeout(uint32_t raw_fault_code) {
    std::lock_guard<std::mutex> lc(variable_mutex);
    if (state_ != DEVICE_ERROR_TIMEOUT) {
        state_ = DEVICE_ERROR_TIMEOUT;
        raw_fault_code_ = raw_fault_code;
        uint32_t old_unicorn_fault_code = unicorn_fault_code_;
        unicorn_fault_code_ = DEVICE_FAULT_TIMEOUT + device_id_ * 1000;
        setUnicornFaultCode(old_unicorn_fault_code,unicorn_fault_code_);
	    updateSrosEmergencySrc(true);
        LOGGER(ERROR, DEVICE) << "Device " << name_ << " timeout, set fault code " << unicorn_fault_code_;
    }
}

void Device::setState(DeviceState state, uint32_t raw_fault_code) {
    switch (state) {
        case DEVICE_NONE: {
            setStateNone();
            break;
        }
        case DEVICE_OK: {
            setStateOK();
            break;
        }
        case DEVICE_INITIALIZING: {
            setStateInitialization();
            break;
        }
        case DEVICE_OFF: {
            setStateOff();
            break;
        }
        case DEVICE_ERROR: {
            setStateError(raw_fault_code);
            break;
        }
        case DEVICE_ERROR_OPEN_FAILED: {
            setStateOpenFailed();
            break;
        }
        case DEVICE_ERROR_TIMEOUT: {
            setStateTimeout();
            break;
        }
        case DEVICE_ERROR_INITIAL: {
            setStateInitialFailed();
            break;
        }
        default: {
            LOG(ERROR) << "illegal state " << state;
            break;
        }
    }
}

void Device::setUnicornFaultCode(uint32_t old_unicorn_fault_code,uint32_t cur_unicorn_fault_code) {
    auto fault_center = sros::core::FaultCenter::getInstance();
    fault_center->removeFault(old_unicorn_fault_code);
    fault_center->addFault(cur_unicorn_fault_code);
}

void Device::cleanFault() {
    auto fault_center = sros::core::FaultCenter::getInstance();
    fault_center->removeFault(unicorn_fault_code_);
    raw_fault_code_ = 0;
    unicorn_fault_code_ = 0;
}

void Device::updateSrosEmergencySrc(bool trigger) {

    // 设置急停触发源
    auto set_em_src_func = [=](bool _trigger,sros::core::SROS_TRIGGER_EMERGENCY_SRC _em_src) -> void {
        if (_trigger) {
            g_state.sros_emergency_src = g_state.sros_emergency_src | _em_src;
        } else {
            g_state.sros_emergency_src = g_state.sros_emergency_src & (~_em_src);
        }
    };

    switch (device_id_) {
        case DEVICE_ID_SRC: {
            set_em_src_func(trigger,sros::core::SROS_EMERGENCY_SRC_TIMEOUT);
            break;
        }
        case DEVICE_ID_LIDAR: {
            set_em_src_func(trigger,sros::core::SROS_EMERGENCY_LASER_TIMEOUT);
            break;
        }
    }
}

void Device::setTimeoutTime(uint32_t ms) {
    CHECK(ms > 0);
    CHECK(ms < 10 * 1000);

    timeout_time_ = ms;
}

std::string Device::getInterfaceName() {
    std::string str = "unknown";

    switch (comm_interface_type_) {
        case DEVICE_COMM_INTERFACE_TYPE_NONE: {
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_ETH_1: {
            str = "ETH1";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_ETH_2: {
            str = "ETH2";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_CAN_1: {
            str = "CAN1";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_CAN_2: {
            str = "CAN2";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS232_1: {
            str = "RS232-1";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS232_2: {
            str = "RS232-2";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS232_3: {
            str = "RS232-3";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS232_4: {
            str = "RS232-4";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS232_5: {
            str = "RS232-5";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS485_1: {
            str = "RS485-1";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_RS485_2: {
            str = "RS485-2";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_USB_0: {
            str = "USB0";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_USB_1: {
            str = "USB1";
            break;
        }
        case DEVICE_COMM_INTERFACE_TYPE_USB_2: {
            str = "USB2";
            break;
        }
        default: {
            break;
        }
    }

    return str;
}

}  // namespace device
}  // namespace sros
