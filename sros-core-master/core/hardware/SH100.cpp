/**
 * @file SH00.cpp
 *
 * @author pengjiali
 * @date 2019年5月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "SH100.h"

#include <glog/logging.h>
#include "core/util/utils.h"

namespace sros {
namespace device {

SH100::SH100(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
             std::shared_ptr<IOInterface> io_interface)
    : SRIODevice(name, device_id, device_comm_interface_type, io_interface) {
    synCmdReset();
    synCmdStart();
}

// void SH100::getTofData(std::vector<TofData> &data) {
//    std::lock_guard<std::mutex> lk(mutex_);
//    data = tof_data_;
//    for (auto tof : tof_data_) {
//        tof.distance = 0;
//        tof.strength = 0;
//    }
//}

void SH100::onDataRecive(const std::vector<uint8_t> &data) {
    //    LOG(INFO) << numberListToStr(data.cbegin(), data.cend());
    if (is_wating_for_response_ && data[0] == request_[0]) {  // conform frame
        setResponse(true);
        LOG(INFO) <<"request_[0]="<<request_[0] <<" data: "<<numberListToStr(data.cbegin(), data.cend());
        //解析设备模式
        if (data[3] == 0x04) {
            dev_model_variable_.setResult(data[4]);
        } else if (data[3] == 0x05) {
            dev_version_variable_.setResult(data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24));
        }
    } else {
        auto dev_addr = data[0];
        if (dev_addr >= 0 && dev_addr <= 0x2f) {  // 前面四个tof的数据
            if (new_tof_data_callback_) {
                uint16_t distance = ((data[1] << 8) + data[2]);
                uint16_t strength = ((data[3] << 8) + data[4]);
                new_tof_data_callback_(dev_addr, distance, strength);
            }
        }
    }
}

bool SH100::syncSetCanPeriod(uint16_t period_ms) {
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
        request_[0] = 0xB2;
        request_[1] = 0x40;
        request_[2] = 0x60;
        request_[3] = 0x03;
        request_[4] = period_ms >> 8;
        request_[5] = period_ms & 0x00ff;
    };
    setRequest(fun);
    syncRequest();

    synCmdStart();

    return false;
}

int SH100::getDeviceModel() {
    dev_model_variable_.reset();
    int iModel = 0;
    auto fun = [&] { request_ = {0x2F, 0x40, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        if (dev_model_variable_.waitForResult(1000,iModel))
        {
            LOG(WARNING) << getName() << " getDeviceModel " << iModel;
            return iModel;
        }
    }
    LOG(WARNING) << getName() << "获取设备型号失败!"; 
    return iModel;
}

std::string SH100::getDeviceModelStr() {
    dev_model_variable_.reset();
    int iModel = getDeviceModel();
    std::string model_num_str = "-";
    if((dev_model)iModel == SR_DEV_SH200) {
        model_num_str = "SR_DEV_SH200";
    } else if ((dev_model)iModel == SR_DEV_SH100) {
        model_num_str = "SR_DEV_SH100";
    }
    return model_num_str;
}

int SH100::getDeviceVersion() {
    dev_version_variable_.reset();
    int iModel = 0;
    auto fun = [&] { request_ = {0x2F, 0x40, 0x60, 0x05, 0x00, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        if (dev_version_variable_.waitForResult(1000,iModel))
        {
            LOG(WARNING) << getName() << " getDeviceVersion " << iModel;
            return iModel;
        }
    }
    LOG(WARNING) << getName() << "获取设备版本号失败!";
    return iModel;
}

std::string SH100::getDeviceVersionStr() {
    dev_version_variable_.reset();
    int iModel = getDeviceVersion();
    int major_version = iModel / (1000 * 1000);
    int minor_version = (iModel / 1000) % 1000;
    int patch_version = iModel % 1000;
    std::string sw_version_no_str =
        std::to_string(major_version) + "." + std::to_string(minor_version) + "." + std::to_string(patch_version);
    return sw_version_no_str;
}


}  // namespace device
}  // namespace sros
