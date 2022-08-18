/**
 * 
 * @copyright   : Copyright (C) 2019 Standard-robots, Inc
 * @file        : IAP_CAN.cpp
 * @description : 
 * @author      : EHL (linenhui@standard-robots.com / enhuilyn@qq.com)
 * @date        : 2022/04/27
 * @brief       : V1.0.0 
 */

#include "IAP_CAN.h"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg_bus.h"

namespace sros {
namespace device {

IapCan::IapCan(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                       std::shared_ptr<IOInterface> io_interface)
    : SRIODevice(name, device_id, device_comm_interface_type, io_interface) {
    LOG(INFO)<<"init";
}

void IapCan::init(const uint32_t dev_type, const uint32_t dev_offset, const uint step)
{
    iap_dev_type_ = dev_type;
    iap_dev_offset_ = dev_offset;
    iap_cmd_step_ = step;
    for(int i = 0; i < 6; i++)
    {
        if(IAP_DEV[i][0] == dev_type) {
            dev_default_id_ = IAP_DEV[i][2];
            break;
        }
    }
    setTimeoutTime(9*1000);
}

bool IapCan::asyncRequestData() {
    std::vector<uint8_t> payload;
    payload.push_back(0x59);
    return asyncRequest(payload);
}

bool IapCan::asyncRequestData(const std::vector<uint8_t> &data) {
    ///LOG(INFO) << "asyncRequestData size = " << data.size();
    if(data.size() > 8) {
        LOG(WARNING) << "cannot send more than 8 bytes error!!!";
        return false;
    }
    return asyncRequest(data);
}

bool IapCan::sendData(const std::vector<uint8_t> &data) {
    return asyncRequestData(data);
}

void IapCan::onDataRecive(const std::vector<uint8_t> &data) {
    LOG(INFO) << "cmd id:" << iap_cmd_step_  << " name:" << IAP_NAME[iap_cmd_step_]  << " data size: "<<data.size() << std::hex <<" data: "<<numberListToStr(data.cbegin(), data.cend());

    if (is_wating_for_response_) {
        setResponse(true);
        if(IAP_HOST_SCAN == iap_cmd_step_)
        {
            version_num_ = data[2] * 1000 * 1000 + data[3] * 1000 + data[4];
            version_str_ = std::to_string(data[2]) + "." + std::to_string(data[3]) + "." + std::to_string(data[4]);
            int offset = data[5] + ((uint32_t)data[6] << 8);
            LOG(INFO) << "device response mode (1:app; 0:bootloader) = " << data[1];
            if(dev_default_id_ == offset)
            {
                dev_response_.setResult(data[1]);
            }
            LOG(INFO) <<"version_num_="<<version_num_ <<" version_str_="<<version_str_<<std::hex<<" dev_default_id_= 0x"<<dev_default_id_<<" ?= 0x"<<offset;
        } else if(IAP_HOST_REQUEST == iap_cmd_step_) {
            if(data.size() > 2) {
                uint32_t dev_id = data[1] + (data[2]<<8);
                LOG(INFO) << "response : " << data[0] << "device id: 0x"<<std::hex<<dev_id;
                dev_response_.setResult(data[0]);
            } else if((data[0] == 0x80) && (data[1] == 0x01)) {
                LOG(INFO)<<"request error";
                dev_response_.setResult(-1);
            }
        } else if((IAP_HOST_SEND_DATA_INFO == iap_cmd_step_) || (IAP_HOST_SEND_FILE_INFO == iap_cmd_step_)) {
            if(data.size() > 2) {
                uint32_t response = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24);
                dev_response_.setResult(response);
                if(iap_data_callback_) {
                    iap_data_callback_(response);
                }
            } else {
                if(0x01 == data[1]) {
                    LOG(ERROR) << "device get pkg data timeout error!!!";
                } else if(0x02 == data[1]) {
                    LOG(ERROR) << "device get pkg data invalid error!!!";
                } else if(0x03 == data[1]) {
                    LOG(ERROR) << "device get pkg data index error!!!";
                }
                dev_response_.setResult(-1);
            }
        } else if(IAP_HOST_SEND_TEST == iap_cmd_step_) {
            uint32_t type = data[1] + ((uint32_t)data[2] << 8);
            if(type == iap_dev_type_)
            {
                dev_response_.setResult(1);
                if(iap_data_callback_) {
                    iap_data_callback_(1);
                }
            }
        }
    } else if(IAP_HOST_SEND_DATA == iap_cmd_step_) {
        if((data.size() == 2) && (data[0] == 0x80)) {
            LOG(INFO)<<"error data size:"<<data.size()<<" "<<data[0]<<" "<<data[1];
            if(iap_data_callback_) {
                iap_data_callback_(-1);
            }
            setResponse(true);
            dev_response_.setResult(-1);
        } else {
            uint32_t ret = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24);
            LOG(INFO)<<"got confirm  = " << ret;
            if(iap_data_callback_) {
                iap_data_callback_(ret);
            }
            setResponse(true);
            dev_response_.setResult(ret);
        }
    } else {
        if(iap_data_callback_) {
            if(data.size() >= 4) {
                iap_data_callback_(data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24));
            } else if(data.size() == 2) {
                iap_data_callback_(data[0] + (data[1]<<8));
            } else {
               iap_data_callback_(data[0]); 
            }
        }
    }
}

int IapCan::scanDevice(const uint32_t dev_type) {
    iap_cmd_step_ = IAP_HOST_SCAN;
    iap_dev_type_ = dev_type;
    dev_response_.reset();
    int ret = -1;
    auto fun = [&] { request_ = {0x1, dev_default_id_&0xFF, (dev_default_id_>>8)&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        if (dev_response_.waitForResult(1000,ret))
        {
            LOG(WARNING) << getName() << " scanDevice " << ret;
            return ret;
        }
    }
    LOG(WARNING) << getName() << "扫描设备失败!";
    return ret;
}

int IapCan::requestDeviceEnterIAP() {
    iap_cmd_step_ = IAP_HOST_REQUEST;
    dev_response_.reset();
    int ret = -1;
    auto fun = [&] { request_ = {dev_default_id_&0xFF, (dev_default_id_>>8)&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        if (dev_response_.waitForResult(500,ret))
        {
            LOG(WARNING) << getName() << " getDeviceResponse " << ret;
            return ret;
        }
    }
    LOG(WARNING) << getName() << "请求设备进入IAP模式失败!";
    return ret;
}

int IapCan::sendFileInfo(uint32_t pkgNum) {
    LOG(INFO) << "sendFileInfo ...";
    iap_cmd_step_ = IAP_HOST_SEND_FILE_INFO;
    dev_response_.reset();
    int ret = -1;
    auto fun = [&] { request_ = {pkgNum&0xFF, (pkgNum>>8)&0xFF, (pkgNum>>16)&0xFF, (pkgNum>>24)&0xFF, 0x00, 0x00, 0x00, 0x00}; };
    setRequest(fun);
    if (syncRequest()) {
        if (dev_response_.waitForResult(1000,ret))
        {
            LOG(WARNING) << getName() << " sendFileInfo ret = " << ret;
            if(ret > pkgNum) {
                LOG(ERROR) << getName() << "pkgNum = " << pkgNum << " ret = " << ret;
                ret = -1;
            }
            return ret;
        }
    }
    LOG(WARNING) << getName() << "sendFileInfo error!!!";
    return ret;
}

int IapCan::sendDataPacketInfo(uint32_t packetIndex,  const uint32_t pkgDataSize, uint32_t crc) {
    LOG(INFO) << "send data pkg info ...";
    iap_cmd_step_ = IAP_HOST_SEND_DATA_INFO;
    dev_response_.reset();
    int ret = -1;
    auto fun = [&] { request_ = {packetIndex&0xFF, (packetIndex>>8)&0xFF, (packetIndex>>16)&0xFF, (packetIndex>>24)&0xFF,pkgDataSize&0xFF, (pkgDataSize>>8)&0xFF, (crc)&0xFF, (crc>>8)&0xFF}; };
    setRequest(fun);
    if (syncRequest()) {
        if (dev_response_.waitForResult(1000,ret))
        {
            LOG(WARNING) << getName() << " sendDataPacketInfo ret = " << ret;
            return ret;
        }
    }
    LOG(WARNING) << getName() << " sendDataPacketInfo error !!!";
    return ret;
}

// CAN 每次发送最多 8字节, 设备回复的索引是以8为单位的,每收到一个索引就加1.
bool IapCan::sendIAPdata(const std::vector<uint8_t> &data) {
    iap_cmd_step_ = IAP_HOST_SEND_DATA;
    const uint32_t MAX_PACKAGE_SIZE = 8;
    uint32_t total_send_size = data.size();
    dev_response_.reset();
    LOG(INFO) << "total_send_size = " << total_send_size;
    if (total_send_size) {
        uint32_t index = 0;
        uint32_t length = 0;
        while(index < total_send_size) {
            length = total_send_size - index;
            length = (length > 8) ? 8 : length;
            std::vector<uint8_t> temp;
            for(int i = 0; i < length; i++) {
                temp.push_back(data[i + index]);
            }
            //LOG(INFO) << "index = " << index << " length = " << length << " size of temp = " << temp.size();
            if(!sendData(temp)) {
                break;
            }
            usleep(5*1000);
            index += length;
        }
        LOG(INFO) << "end data index = " << index;
        int ret = 0;
        if (dev_response_.waitForResult(100,ret)) {
            LOG(WARNING) << getName() << " sendIAPdata ret = " << ret;
        } else {
            LOG(WARNING) << getName() << " sendIAPdata get ret error!!!";
            return false;
        }
        if(index >= total_send_size) {
            LOG(INFO) << "send data success";
            return true;
        }
    }
    return false;
}

uint32_t IapCan::getDevVersionNo()
{
    return version_num_;
}

std::string IapCan::getDevVersionStr()
{
    return version_str_;
}


}  // namespace device
}  // namespace sros
