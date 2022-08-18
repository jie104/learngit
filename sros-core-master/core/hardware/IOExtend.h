/**
 * @file IOExtend.h
 *
 * @author perry
 * @date 2022-05-18
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef IOEXTEND_H
#define IOEXTEND_H

#include "core/device/virtual_device.hpp"

namespace sros {
namespace device {
class IOExtend : public VirtualDevice {
 public:
    IOExtend(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
            DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~IOExtend() = default;

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;
};
}  // namespace device
}  // namespace sros

#endif  // IOEXTEND_H
