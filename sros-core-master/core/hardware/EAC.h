/**
 * @file EAC.h
 *
 * @author pengjiali
 * @date 2019年5月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_HARDWARE_EAC_H_
#define CORE_HARDWARE_EAC_H_

#include "core/device/virtual_device.hpp"

namespace sros {
namespace device {

class EAC : public VirtualDevice {
 public:
    EAC(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
        DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~EAC() = default;

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;
};

typedef std::shared_ptr<EAC> EAC_ptr;
}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_EAC_H_
