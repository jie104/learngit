/**
 * @file can_interface.cpp
 *
 * @author pengjiali
 * @date 2019年05月20日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "can_interface.h"
#include "core/usart/canbus.h"

namespace sros {
namespace device {

CanInterface::CanInterface(uint32_t can_id, uint32_t response_id) : can_id_(can_id), response_id_(response_id) {}

bool CanInterface::write(const std::vector<uint8_t> &data) {
    Canbus::getInstance().sendData(can_id_, data);
    return true;
}

void CanInterface::setReadCallback(IOInterfaceReadDataFunc func) {
    Canbus::getInstance().addDevice(response_id_, func);
}

}  // namespace device
}  // namespace sros
