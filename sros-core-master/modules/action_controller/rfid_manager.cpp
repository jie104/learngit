/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#include "rfid_manager.h"

#include <vector>

#include "glog/logging.h"
#include "rfid_protocol_wyuan.h"
#include "rfid_protocol_pepperl_fuchs.h"
#include "core/settings.h"

namespace ac {

RfidManager* RfidManager::getInstance() {
    static RfidManager instance;
    return &instance;
}

bool RfidManager::init(std::string uart_name, unsigned int baud_rate) {
    auto &s = sros::core::Settings::getInstance();
    auto rfid_mfrs_name = s.getValue<std::string>("device.rfid_mfrs_name", MFRS_GUANGZHOU_WYUAN);
    LOG(INFO) << "rfid, mfrs_name:" << rfid_mfrs_name << ",uart_name:" << uart_name << ",baud_rate:" << baud_rate;

    if(rfid_mfrs_name == MFRS_GUANGZHOU_WYUAN) {
        rfid_mfrs_name_ = MFRS_GUANGZHOU_WYUAN;
        rfid_prot_interface_ptr_ = std::make_shared<RfidProtWYuan>();
    } else if(rfid_mfrs_name == MFRS_PEPPERL_FUCHS) {
        rfid_mfrs_name_ = MFRS_PEPPERL_FUCHS;
        rfid_prot_interface_ptr_ = std::make_shared<RfidProtPepperlFuchs>();
    } else {
        LOG(ERROR) << "can not support rfid mfrs: " << rfid_mfrs_name;
        return false;
    }

    if(!connect(uart_name, baud_rate)) {
        LOG(ERROR) << "open uart fial, uart_name:" << uart_name << ", baud_rate:" << baud_rate;
        return false;
    }

    //初始化
    bool ret = rfid_prot_interface_ptr_->init();

    if(!ret) {
        return false;
    }

    //打开串口成功
    connected_ = true;
    return true;

}

bool RfidManager::connect(std::string uart_name, unsigned int baud_rate, bool rs485_mode) {
    serial_ptr_ = std::make_shared<CallbackAsyncSerial>();

    serial_ptr_->open(uart_name, baud_rate, rs485_mode);
    serial_ptr_->setCallback(boost::bind(&RfidManager::onRecvSerialData, this, _1, _2));

    return serial_ptr_->isOpen();
}

bool RfidManager::disconnect() {
    if(serial_ptr_ && serial_ptr_->isOpen()) {
        serial_ptr_->close();
        return true;
    }

    return false;
}

void RfidManager::asyncGetRfidCmd() {
    std::string rfid_data;
    if(!connected_) {
        LOG(ERROR) << "uart disconnect";
        return;
    }

    if(rfid_prot_interface_ptr_) {
        rfid_prot_interface_ptr_->asyncGetRfidCmd();
    }

    return;
}

std::string RfidManager::asyncGetRfidData() {
    std::string rfid_data;
    if(!connected_) {
        LOG(ERROR) << "uart disconnect";
        return rfid_data;
    }

    if(rfid_prot_interface_ptr_) {
        return rfid_prot_interface_ptr_->asyncGetRfidData();
    }

    return rfid_data;
}

std::string RfidManager::syncGetRfid() {
    std::string rfid_data;
    if(!connected_) {
        LOG(ERROR) << "uart disconnect";
        return rfid_data;
    }

    if(rfid_prot_interface_ptr_) {
        return rfid_prot_interface_ptr_->getRfid();
    }

    return rfid_data;
}

bool RfidManager::sendData(const std::vector<uint8_t>& data) {
    if(serial_ptr_ && serial_ptr_->isOpen()) {
        serial_ptr_->write((char *) (data.data()), data.size());
        return true;
    }
    return false;
}


void RfidManager::onRecvSerialData(const char* data, size_t size) {
    std::vector<uint8_t> vecData(size);
    for(int i = 0; i < size; ++i) {
        vecData[i] = data[i];
    }

    if(rfid_prot_interface_ptr_) {
        rfid_prot_interface_ptr_->parseMsg(vecData);
    }
}


}

