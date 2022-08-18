/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#include "rfid_protocol_wyuan.h"
#include "rfid_manager.h"

namespace ac {
    
RfidProtWYuan::RfidProtWYuan():RfidProtInterface() {}

RfidProtWYuan::~RfidProtWYuan() {}

bool RfidProtWYuan::init() {
    return true;
}

void RfidProtWYuan::parseMsg(std::vector<uint8_t>& data) {
    if(feedReciveRawData(data)) {
        if(promise_ptr_) {
            promise_ptr_->set_value(std::vector<uint8_t>());
        }
    }
}

std::string RfidProtWYuan::getRfid() {
    
    std::string rfid_data;
    setQuerySingleLableCmd();

    //等待数据
    promise_ptr_ = std::make_shared<std::promise<std::vector<uint8_t>>>();
    auto future = promise_ptr_->get_future();

    RfidManager::getInstance()->sendData(getSendRawData());

    auto status = future.wait_for(std::chrono::milliseconds(sync_wait_time_)); 
    if(status == std::future_status::timeout) {
        LOG(ERROR) << "recv timeout";
        return rfid_data;
    }

    std::vector<uint8_t> raw_data;
    raw_data = getUID();
    if(raw_data.empty()) {
        LOG(ERROR) << "recv data is empty";
        return rfid_data;
    }

    rfid_data = vecToString(raw_data);
    LOG(INFO) << "get rfid: " <<  rfid_data;

    return rfid_data;
}

void RfidProtWYuan::setQuerySingleLableCmd() {
    recv_raw_data_.clear();
    send_raw_data_ = {0x04, 0xFF, 0x50, 0x17, 0xF7};
}


std::vector<uint8_t> RfidProtWYuan::getUID() {
    std::vector<uint8_t> uid;

    if (recv_raw_data_.empty()) {
        LOG(INFO) << "data is empty!";
        return uid;
    }

    if (recv_raw_data_[RECIVEFILD_STATUS] != ResultStatus::SUCCEED) {
        LOG(INFO) << "Got UUID failed， status：" << (int)recv_raw_data_[RECIVEFILD_STATUS];
        return uid;
    }

    auto uid_it = recv_raw_data_.begin() + RECIVEFILD_DATA;
    uid.resize(8);
    std::copy(uid_it, uid_it + RFID_UID_LEN, uid.begin());

    return uid;

}

void RfidProtWYuan::setReadDataCmd(const std::vector<uint8_t> &uid) {
    recv_raw_data_.clear();
    send_raw_data_ = {0x0e, 0x00, 0x52, 0x08};
    send_raw_data_.resize(send_raw_data_.size() + RFID_UID_LEN);
    std::copy(uid.begin(), uid.end(), send_raw_data_.begin() + 4);
    send_raw_data_.push_back(READ_DATA_COUNT);
    uint16_t crc16 = uiCrc16Cal(send_raw_data_);
    send_raw_data_.push_back(crc16 & 0xFF);
    send_raw_data_.push_back((crc16 >> 8) & 0xFF);
}

std::vector<uint8_t> RfidProtWYuan::getReadData() {
    std::vector<uint8_t> data;

    if (recv_raw_data_.empty()) {
        return data;
    }

    if (recv_raw_data_[RECIVEFILD_STATUS] != ResultStatus::SUCCEED) {
        LOG(INFO) << recv_raw_data_[RECIVEFILD_STATUS]; // 0xFB（251）表示没有可用的RFID
        return data;
    }

    auto data_it = recv_raw_data_.begin() + RECIVEFILD_DATA;
    data.resize(READ_DATA_COUNT);
    std::copy(data_it, data_it + READ_DATA_COUNT, data.begin()); // 读取10位

    return data;
}

const std::vector<uint8_t> &RfidProtWYuan::getSendRawData() {
    return send_raw_data_;
}

bool RfidProtWYuan::feedReciveRawData(const std::vector<uint8_t> &data) {
    recv_raw_data_.insert(recv_raw_data_.end(), data.begin(), data.end());

    if (recv_raw_data_.empty()) {
        LOG(INFO) << "data is empty!";
        return false;
    }

    if (recv_raw_data_[RECIVEFILD_LEN] + 1 != recv_raw_data_.size()) {
        LOG(INFO) << "Length check is incorrect!";
        return false;
    }

    if (recv_raw_data_[RECIVEFILD_RECMD] != send_raw_data_[SENDFIELD_CMD]) {
        LOG(INFO) << "CMD missmatch!";
        return false;
    }

    if (uiCrc16Cal(recv_raw_data_) != 0x0000) {
        LOG(INFO) << "RCR check failed!";
        return false;
    }

    return true;
}

uint16_t RfidProtWYuan::uiCrc16Cal(const std::vector<uint8_t> &data) {
    const uint32_t PRESET_VALUE = 0xFFFF;
    const uint32_t POLYNOMIAL = 0x8408;

    uint8_t ucJ;
    uint16_t uiCrcValue = PRESET_VALUE;

    for (auto it = data.cbegin(); it != data.cend(); ++it) {
        uiCrcValue = uiCrcValue ^ *it;
        for (ucJ = 0; ucJ < 8; ucJ++) {
            if (uiCrcValue & 0x0001) {
                uiCrcValue = (uiCrcValue >> 1) ^ POLYNOMIAL;
            } else {
                uiCrcValue = (uiCrcValue >> 1);
            }
        }
    }
    return uiCrcValue;
}


}