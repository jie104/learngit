/**
 * @file R2100.cpp
 *
 * @author pengjiali
 * @date 2019年5月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "R2100.h"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg_bus.h"

namespace sros {
namespace device {

R2100::R2100(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
             std::shared_ptr<IOInterface> io_interface)
    : IODevice(name, device_id, device_comm_interface_type, io_interface) {}

bool R2100::asyncRequestData() {
    std::vector<uint8_t> payload;
    payload.push_back(0x59);
    return asyncRequest(payload);
}

void R2100::onDataRecive(const std::vector<uint8_t> &data) {
    const int LASER_SENSOR_NUM = 11;

    uint8_t command = data[3];

    //    LOG(INFO) << "==> onRecvUsartData:  " << "size = " << data.size();

    if (command == 0x11) {
        // 构造DistanceDataMsg发送
        auto mm = std::make_shared<sros::core::DistanceDataMsg>("R2100_DATA");
        mm->distances.clear();
        mm->time_ = sros::core::util::get_time_in_us();
        mm->sensor_name = getName();
        const uint32_t INFINITY_DISTANCE = 0xffffffff;
        for (int i = 0, j = 0; j < LASER_SENSOR_NUM && i < data.size(); i += 4, j++) {
            uint32_t d = ((data[4 + i + 1] << 8) + data[4 + i]);  // 单位mm

            // According R2100 manual:
            // If a beam does not detect a target, the corresponding distance and echo values are reported as 0xffff
            if (d >= 0xffff) {
                d = INFINITY_DISTANCE;
            }

            mm->distances.push_back(d);
        }

        sros::core::MsgBus::sendMsg(mm);
    }
}
}  // namespace device
}  // namespace sros
