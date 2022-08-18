/**
 * @file SH200.cpp
 *
 * @author perry
 * @date 2021年9月13日
 *
 * @describe
 *
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */

#include "SH200.h"

#include <glog/logging.h>
#include "core/util/utils.h"
#include "core/settings.h"

namespace sros {
namespace device {

SH200::SH200(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
             std::shared_ptr<IOInterface> io_interface)
    : SRIODevice(name, device_id, device_comm_interface_type, io_interface) {
    //设备激活放到初始化完所有设备之后，否则会导致数据回调时，设备取值为空从而导致崩溃
    closeDevice();
}

void SH200::onDataRecive(const std::vector<uint8_t> &data) {
    if (data.size() < 8)
    {
        return;
    }

    if (is_wating_for_response_ && (data[0] == 0x60 || data[0] == 0x80)) {  // conform frame
        setResponse(true);
        LOG(INFO) <<"request_[0]="<<request_[0] <<" data: "<<numberListToStr(data.cbegin(), data.cend());
        //解析设备模式
       // if (data[0] == 0x60 && data[3] == 0x04) // TODO: test SH200 hardware on board
        if (data[3] == 0x04)
        {
            dev_model_variable_.setResult(data[4]);
        } else if (data[3] == 0x03) {
            dev_version_variable_.setResult(data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24));
        }
    } else {
        auto &s = sros::core::Settings::getInstance();
        auto config_tof_num = s.getValue<std::string>("sh100.each_sh100_tof_num", "4tofs_each_sh100");
        int each_shxxx_tof_num = (config_tof_num == "6tofs_each_sh100") ? 6 : 4;

        auto cur_tof_id = data[0];
        auto cur_tof_status = data[7];
        if (cur_tof_id > 0 && cur_tof_id <= each_shxxx_tof_num) {
            if (sh200_tof_data_callback_ && !cur_tof_status) {
                uint16_t distance = ((data[2] << 8) + data[1]);
                uint16_t strength = (data[6] << 0x18) + (data[5] << 0x10) + (data[4] << 8) + data[3];
                sh200_tof_data_callback_(cur_tof_id, distance, strength);
            }
        }
    }
}

void SH200::initProtocol()
{
    dev_model eModel = (dev_model)getDeviceModel();
    if (eModel == SR_DEV_SH200)
    {
        //先关闭设备
        closeDevice();

        //进入配置模式
        enterConfigMode();

        //TOD(LSP)配置TOF类型
        auto &s = sros::core::Settings::getInstance();
        auto shxxx_tof_type = s.getValue<std::string>("device.shxxx_tof_type", "SR_NULL");
        if (shxxx_tof_type == "SR_VL53L1X") {
            setTofDevType(SR_TYPE_VL53L1X);
        }
        else if(shxxx_tof_type == "SR_TFMINI_PLUS") {
            setTofDevType(SR_TYPE_TFMINI_PLUS);
        }
        else {
            setTofDevType(SR_TYPE_NULL);
        }

        //配置TOF数量
        auto tof_num = s.getValue<std::string>("sh100.each_sh100_tof_num", "4tofs_each_sh100");
        int each_shxxx_tof_num = (tof_num == "6tofs_each_sh100") ? 6 : 4;
        setTofNums(each_shxxx_tof_num);

        //退出配置模式
        leaveConfigMode();

        //激活设备
        openDevice();
    }
    else {
        LOG(WARNING) << getName() << "获取设备型号异常!(非sh200)";
    }
}

bool SH200::openDevice() {
    auto fun = [&] { request_ = {0x2F, 0x11, 0x20, 0x01, 0x01, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    setStateInitialFailed();
    return false;
}

bool SH200::closeDevice() {
    auto fun = [&] { request_ = {0x2F, 0x11, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    LOG(WARNING) << getName() << "关闭设备失败!";
    return false;
}

int SH200::getDeviceModel() {
    dev_model_variable_.reset();
    int iModel = 0;
    auto fun = [&] { request_ = {0x2F, 0x11, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00}; };
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

std::string SH200::getDeviceModelStr() {
    int iModel = getDeviceModel();
    std::string model_num_str = "-";
    if((dev_model)iModel == SR_DEV_SH200) {
        model_num_str = "SR_DEV_SH200";
    } else if ((dev_model)iModel == SR_DEV_SH100) {
        model_num_str = "SR_DEV_SH100";
    }
    return model_num_str;
}

int SH200::getDeviceVersion() {
    dev_version_variable_.reset();
    int iModel = 0;
    auto fun = [&] { request_ = {0x2F, 0x11, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00}; };
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

std::string SH200::getDeviceVersionStr() {
    int iModel = getDeviceVersion();
    int major_version = iModel / (1000 * 1000);
    int minor_version = (iModel / 1000) % 1000;
    int patch_version = iModel % 1000;
    std::string sw_version_no_str =
        std::to_string(major_version) + "." + std::to_string(minor_version) + "." + std::to_string(patch_version);
    return sw_version_no_str;
}

bool SH200::enterConfigMode() {
    auto fun = [&] { request_ = {0x2F, 0x01, 0x20, 0x01, 0x01, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    LOG(WARNING) << getName() << "进入配置模式失败!";
    return false;
}

bool SH200::leaveConfigMode() {
    auto fun = [&] { request_ = {0x2F, 0x01, 0x20, 0x02, 0x01, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    LOG(WARNING) << getName() << "退出配置模式失败!";
    return false;
}

bool SH200::setTofNums(uint8_t tof_nums)
{
    auto fun = [&] { request_ = {0x2F, 0x01, 0x20, 0x09, tof_nums, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    LOG(WARNING) << getName() << "配置TOF数量失败!";
    return false;
}

bool SH200::setTofDevType(tof_type tof_type)
{
    auto fun = [&] { request_ = {0x2F, 0x01, 0x20, 0x03, (uint8_t)tof_type, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    LOG(WARNING) << getName() << "配置TOF设备类型失败!";
    return false;
}

bool SH200::setTofDistMode(tof_dist_mode tof_dist_mode)
{
    auto fun = [&] { request_ = {0x2F, 0x01, 0x20, 0x04, (uint8_t)tof_dist_mode, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        return true;
    }

    LOG(WARNING) << getName() << "配置TOF测距模式失败!";
    return false;
}
}  // namespace device
}  // namespace sros
