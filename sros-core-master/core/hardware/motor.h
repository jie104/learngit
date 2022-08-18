/**
 * @file motor.h
 *
 * @author pengjiali
 * @date 19-9-6.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "core/device/virtual_device.hpp"

namespace sros {
namespace device {

class Motor : public VirtualDevice {
 public:
    Motor(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~Motor() = default;

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;
};

using Motor_ptr = std::shared_ptr<Motor>;

}  // namespace device
}  // namespace sros

#endif  // MOTOR_H
