/**
 * @file device_manager.cpp
 *
 * @author lhx
 * @date 2018年1月31日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <glog/logging.h>
#include <boost/thread/lock_guard.hpp>

#include "core/fault_center.h"
#include "device_manager.h"

namespace sros {
namespace device {

DeviceManager *DeviceManager::getInstance() {
    static DeviceManager manager;
    return &manager;
}

Device_ptr DeviceManager::getDeviceById(int id) const {
    auto devices = std::atomic_load(&devices_);
    for (const auto &p : *devices) {
        auto device = p.second;
        if (device->getID() == id) {
            return p.second;
        }
    }
    return nullptr;
}

Device_ptr DeviceManager::getDeviceByName(const std::string &name) const {
    auto devices = std::atomic_load(&devices_);
    if (!devices || devices->find(name) == devices->end()) {
        return nullptr;
    } else {
        return devices->at(name);
    }
}

Device_ptr DeviceManager::registerDevice(const std::string &name, DeviceID device_id,
                                         DeviceCommInterfaceType device_comm_interface_type,
                                         DeviceMountType device_type) {
    std::lock_guard<std::mutex> lg(mutex_);
    if (getDeviceByName(name)) {
        LOG(WARNING) << "registerDevice() : device " << name << " is exists";
        return nullptr;
    }

    auto device = std::make_shared<Device>(name, device_id, device_comm_interface_type, device_type);

    auto devices = std::atomic_load(&devices_);
    devices->insert(std::make_pair(name, device));
    std::atomic_store(&devices_, devices);

    //    LOG(INFO) << "registerDevice() : device " << name << " registered.";

    return device;
}

bool DeviceManager::addDevice(Device_ptr device) {
    std::lock_guard<std::mutex> lg(mutex_);
    if (getDeviceByName(device->getName())) {
        LOG(WARNING) << "registerDevice() : device " << device->getName() << " is exists";
        return false;
    }
    auto devices = std::atomic_load(&devices_);
    devices->insert(std::make_pair(device->getName(), device));
    std::atomic_store(&devices_, devices);

    return true;
}

std::shared_ptr<const std::map<std::string, Device_ptr>> DeviceManager::getDeviceList() const {
    return std::const_pointer_cast<const std::map<std::string, Device_ptr>>(
        std::atomic_load(&devices_));
}

bool DeviceManager::isDeviceOK(const std::string &device_name) const {
    auto device = getDeviceByName(device_name);

    if (device) {
        return device->isOk();
    }

    return true;  // 若找不到设备也认为是好使的，比如说没有安装这个设备，我们就不需要检测这个设备
}

}  // namespace device
}  // namespace sros
