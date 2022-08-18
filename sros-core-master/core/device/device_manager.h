/**
 * @file device_manager.h
 *
 * @author lhx
 * @date 2018年1月31日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_DEVICE_DEVICE_MANAGER_H_
#define CORE_DEVICE_DEVICE_MANAGER_H_

#include <map>
#include <mutex>

#include "device.h"

namespace sros {
namespace device {

class DeviceManager {
 public:
    static DeviceManager *getInstance();

    Device_ptr registerDevice(const std::string &name, DeviceID device_id,
                              DeviceCommInterfaceType device_comm_interface_type, DeviceMountType device_type);

    bool addDevice(Device_ptr device);

    Device_ptr getDeviceById(int id) const;
    Device_ptr getDeviceByName(const std::string &name) const;

    std::shared_ptr<const std::map<std::string, Device_ptr>> getDeviceList() const;

    bool isDeviceOK(const std::string &device_name) const;

 private:
    DeviceManager() { devices_ = std::make_shared<std::map<std::string, Device_ptr>>(); }

    std::mutex mutex_;  // 这个锁专门锁Fault_list,只是添加错误和删除错误的时候需要添加，读取不要加锁，用atomic_load。
    std::shared_ptr<std::map<std::string, Device_ptr>> devices_;  // 触发的故障, 最严重的故障放到最前
};

}  // namespace device
}  // namespace sros

#endif  // CORE_DEVICE_DEVICE_MANAGER_H_
