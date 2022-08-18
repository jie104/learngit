/**
 * @file Encoder.h
 *
 * @author perry
 * @date 2022-05-18
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "core/device/virtual_device.hpp"

namespace sros {
namespace device {
class Encoder : public VirtualDevice {
 public:
    Encoder(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
            DeviceMountType device_type)
        : VirtualDevice(name, device_id, device_comm_interface_type, device_type) {}
    virtual ~Encoder() = default;

    // 设置编码器位置值
    void setPosition(const int32_t _iPos);

    // 获取编码器位置值
    int32_t getPosition();

 protected:
    uint32_t faultCodeMapping(uint32_t raw_fault_code) final;

private:
    int32_t iPosition = 0;
};
}  // namespace device
}  // namespace sros

#endif  // ENCODER_H
