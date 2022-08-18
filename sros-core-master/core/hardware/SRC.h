/**
 * @file SRC.h
 *
 * @author caoyan
 * @date 2020/11/6
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_HARDWARE_SRC_H_
#define CORE_HARDWARE_SRC_H_

#include "core/device/virtual_device.hpp"

namespace sros {
namespace device {

class SRC : public VirtualDevice {
 public:
    SRC(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
        DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~SRC() = default;

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;
};

typedef std::shared_ptr<SRC> SRC_ptr;
}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_SRC_H_
