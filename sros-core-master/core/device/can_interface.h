/**
 * @file can_interface.h
 *
 * @author pengjiali
 * @date 2019年05月20日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_DEVICE_CAN_INTERFACE_H_
#define CORE_DEVICE_CAN_INTERFACE_H_

#include "IOInterface.hpp"

namespace sros {
namespace device {

class CanInterface : public IOInterface {
 public:
    explicit CanInterface(uint32_t can_id) : CanInterface(can_id, can_id + 0x100) {}

    CanInterface(uint32_t can_id, uint32_t response_id);

    bool isCanInterface() const final { return true; }  // 是否是can接口
 protected:
    void setReadCallback(IOInterfaceReadDataFunc func) final;

    bool write(const std::vector<uint8_t> &data) final;

 private:
    uint32_t can_id_ = 0;       // 设备ID
    uint32_t response_id_ = 0;  // 设备回复的ID
};

}  // namespace device
}  // namespace sros

#endif  // CORE_DEVICE_CAN_INTERFACE_H_
