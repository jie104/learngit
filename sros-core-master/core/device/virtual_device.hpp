/**
 * @file virtual_device.h
 *
 * @author pengjiali
 * @date 19-9-6.
 *
 * @describe 相对于sros来说算是虚拟设备，如电机、IMU等，他们没有IO
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef VIRTUAL_DEVICE_HPP
#define VIRTUAL_DEVICE_HPP

#include "device.h"
#include "core/logger.h"
#include "core/device/device_manager.h"

namespace sros {
namespace device {

class VirtualDevice : public Device {
 public:
    VirtualDevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                  DeviceMountType device_type)
        : Device(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~VirtualDevice() = default;
};

template <class DeviceType>
std::shared_ptr<DeviceType> createDevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                                         DeviceMountType device_type) {
    if (DeviceManager::getInstance()->getDeviceByName(name)) {
        LOGGER(FATAL, DEVICE) << "Register device failed! device " << name << " is exists!";
        return nullptr;
    }

    auto device = std::make_shared<DeviceType>(name, device_id, device_comm_interface_type, device_type);
    DeviceManager::getInstance()->addDevice(device);
    return device;
}

typedef std::shared_ptr<VirtualDevice> VirtualDevice_ptr;
}  // namespace device
}  // namespace sros

#endif  // VIRTUAL_DEVICE_HPP
