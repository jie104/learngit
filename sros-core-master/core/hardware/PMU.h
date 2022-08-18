/**
 * @file PMU.h
 *
 * @author pengjiali
 * @date 19-9-7.
 *
 * @describe 管理电池的控制器
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PMU_H
#define PMU_H

#include "core/device/virtual_device.hpp"

namespace sros {
namespace device {
class PMU : public VirtualDevice {
    PMU(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~PMU() = default;

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;
};
}
}

#endif  // PMU_H
