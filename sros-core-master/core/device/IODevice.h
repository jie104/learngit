/**
 * @file IODevice.h
 *
 * @author pengjiali
 * @date 2019年5月20日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_DEVICE_IODEVICE_H_
#define CORE_DEVICE_IODEVICE_H_

#include <glog/logging.h>
#include <memory>
#include <vector>
#include "IOInterface.hpp"
#include "core/util/async_condition_variable.hpp"
#include "device.h"
#include "device_manager.h"
#include "core/logger.h"

namespace sros {
namespace device {

enum LastErrorCode {
    ERROR_CODE_NONE,
    ERROR_CODE_RESPONSE_TIMEOUT,   // 回复超时
    ERROR_CODE_RESPONSE_INORRECT,  // 回复错误
    ERROR_CODE_LENGTH_INCORRECT,   // 长度不正确
    ERROR_CODE_CMD_MISSMATCH,      // cmd不匹配
};

enum SystemState {
    SYSTEM_STATE_NONE = 0x00,
    SYSTEM_STATE_INIT = 0x01,
    SYSTEM_STATE_IDLE = 0x02,
    SYSTEM_STATE_RUNNING = 0x03,
    SYSTEM_STATE_ERROR = 0x04,
};

class IODevice : public Device {
 public:
    IODevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
             std::shared_ptr<IOInterface> io_interface);

    void init(const std::vector<uint8_t> &data);

    void dataReciveCallback(const std::vector<uint8_t> &data);  // 收到数据的回调函数

    bool isWatingForResponse() const { return is_wating_for_response_; }

    LastErrorCode getLastErrorCode() const { return last_error_code_; }

 protected:
    virtual void onDataRecive(const std::vector<uint8_t> &data) = 0;  // 处理收到的数据，各个设备需要重写此函数

    void setRequest(std::function<void()> fun);

    bool syncRequest();                                   // 阻塞等待回复
    bool asyncRequest(const std::vector<uint8_t> &data);  // 只管发，不管收
    void setResponse(bool ok);

    bool is_error_ = false;                            // 标记是否出错
    bool is_wating_for_response_ = false;              // 当前消息是否正在等待回复
    AsyncConditionVariable<bool> async_condition_;     // 发送请求后等待回复
    LastErrorCode last_error_code_ = ERROR_CODE_NONE;  // 上一次失败的错误码
    std::vector<uint8_t> request_;                     // 请求的指令
    std::vector<uint8_t> response_;                    // 回复的指令
    std::shared_ptr<IOInterface> io_interface_ptr_;
};

template <class DeviceType>
std::shared_ptr<DeviceType> createDevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                                         std::shared_ptr<IOInterface> io_interface) {
    if (DeviceManager::getInstance()->getDeviceByName(name)) {
        LOGGER(FATAL, DEVICE) << "Register device failed! device " << name << " is exists!";
        return nullptr;
    }

    auto device = std::make_shared<DeviceType>(name, device_id, device_comm_interface_type, io_interface);
    DeviceManager::getInstance()->addDevice(device);

    if (io_interface->isCanInterface()) {
        device->setStateOK(); // can设备不需要打开设备，直接就能用
    }

    return device;
}

}  // namespace device
}  // namespace sros

#endif  // CORE_DEVICE_IODEVICE_H_
