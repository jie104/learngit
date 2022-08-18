/**
 * @file R2100.h
 *
 * @author pengjiali
 * @date 2019年5月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_HARDWARE_R2100_H_
#define CORE_HARDWARE_R2100_H_

#include "core/device/IODevice.h"

namespace sros {
namespace device {

class R2100 : public IODevice {
 public:
    R2100(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          std::shared_ptr<IOInterface> io_interface);

    bool asyncRequestData();

 protected:
    void onDataRecive(const std::vector<uint8_t> &data) final;  // 处理收到的数据，各个设备需要重写此函数
};

typedef std::shared_ptr<R2100> R2100_ptr;
}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_R2100_H_
