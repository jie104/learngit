/**
 * @file battery.h
 *
 * @author pengjiali
 * @date 19-9-7.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "motor.h"

namespace sros {
namespace device {
class Battery : public VirtualDevice {
 public:
    Battery(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
            DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~Battery() = default;

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;
};
}  // namespace device
}  // namespace sros

#endif  // BATTERY_H
