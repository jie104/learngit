/**
 * @file EU100.cpp
 *
 * @author pengjiali
 * @date 2019年5月19日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "EU100.h"
#include <glog/logging.h>

namespace sros {
namespace device {

EU100::EU100(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
             std::shared_ptr<IOInterface> io_interface)
    : SRIODevice(name, device_id, device_comm_interface_type, io_interface) {
    synCmdReset();
    synCmdStart();
}

// void EU100::getIO(uint8_t &DI, uint8_t &DO) {
//    std::lock_guard<std::mutex> lk(mutex_);
//
//    DI = di_;
//    DO = do_;
//
//    di_ = 0;
//    do_ = 0;
//}

void EU100::onDataRecive(const std::vector<uint8_t> &data) {
    if (data[0] == 0x55 && data[7] == 0x55) {  // conform frame
        eu100_type_ = (EU100Type)data[3];
        setResponse(true);
    } else {
        if (new_eu100_id_data_callback_) {
            new_eu100_id_data_callback_(data[6], data[7]);
        }
    }
}

bool EU100::syncSetCanPeriod(uint16_t period_ms) {
    if (period_ms < 10 || period_ms > 1000) {
        LOG(INFO) << "The parameter range is wrong!";
        return false;
    }

    if (period_ms % 10 != 0) {  // 不是10的倍数
        LOG(INFO) << "The parameter range is not multiple of 10!";
        return false;
    }

    if (!synCmdReset()) {
        LOG(INFO) << "cmd reset failed!";
        return false;
    }

    auto fun = [&] {
        request_ = std::vector<uint8_t>(8, 0);
        request_[0] = 0x72;
        request_[1] = 0x50;
        request_[2] = 0x60;
        request_[3] = 0x00;
        request_[4] = (uint8_t)eu100_type_;
        request_[5] = period_ms >> 8;
        request_[6] = period_ms & 0x00ff;
    };
    setRequest(fun);
    syncRequest();

    synCmdStart();

    return true;
}

void EU100::setIO(uint8_t mask, uint8_t value) {
    request_ = std::vector<uint8_t>(8, 0);
    request_[0] = 0xA0;
    request_[1] = 0x72;
    request_[2] = 0x21;
    request_[3] = 0x64; // modify by zx 2021-1-19: 0x00 -> 0x64
    request_[4] = 0x00;
    request_[5] = 0x00;
    request_[6] = mask;
    request_[7] = value;
    asyncRequest(request_);
}

}  // namespace device
}  // namespace sros
