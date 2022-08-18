/**
 * @file SRIODevice.h
 *
 * @author pengjiali
 * @date 2019年5月27日
 *
 * @describe 将斯坦德设备相同的命令提取出来
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "SRIODevice.h"

namespace sros {
namespace device {

SRIODevice::SRIODevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                       std::shared_ptr<IOInterface> io_interface)
    : IODevice(name, device_id, device_comm_interface_type, io_interface) {}

bool SRIODevice::synCmdStart() {
    auto fun = [&] { request_ = {0xF2, 0x40, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        setStateOK();
        return true;
    }

    setStateInitialFailed();
    return false;
}

bool SRIODevice::synCmdReset() {
    auto fun = [&] { request_ = {0xF2, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {

        return true;
    }

    LOG(WARNING) << getName() << "进入配置模式失败!";
    return false;
}

}  // namespace device
}  // namespace sros
