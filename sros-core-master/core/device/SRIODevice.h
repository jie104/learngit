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
#ifndef CORE_DEVICE_SRIODEVICE_H_
#define CORE_DEVICE_SRIODEVICE_H_

#include "IODevice.h"

namespace sros {
namespace device {

class SRIODevice : public IODevice {
 public:
    SRIODevice(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
               std::shared_ptr<IOInterface> io_interface);

 protected:
    bool synCmdReset();
    bool synCmdStart();
};

}  // namespace device
}  // namespace sros

#endif  // CORE_DEVICE_SRIODEVICE_H_
