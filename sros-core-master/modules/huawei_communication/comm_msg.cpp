//
// describe: 
// Created by pengjiali on 18-11-6.
//

#include "comm_msg.h"

namespace huawei {

void CommMsg::generateRawData() {
    raw_data_.resize(data_.size() + 8);
    raw_data_[(size_t) Field::SOF] = 0x68;
    raw_data_[(size_t) Field::T] = 0x05;
    raw_data_[(size_t) Field::A] = seq_;
    raw_data_[(size_t) Field::D] = 0x01;
    raw_data_[(size_t) Field::L] = data_.size() + 2;
    raw_data_[(size_t) Field::C] = command_;
    raw_data_[raw_data_.size() - 1] = 0xFE;

    std::move(data_.begin(), data_.end(), std::next(raw_data_.begin(), (int) Field::DATA));

    // 校验码
    int sum = 0;
    for (int i = 0; i < raw_data_.size() - 2; ++i) {
        sum += raw_data_[i];
    }
    raw_data_[raw_data_.size() - 2] = sum & 0xFF;
}

bool CommMsg::decodeRawData() {
    if (raw_data_.size() < 9 || raw_data_.size() > 255 + 8) {
        return false;
    }

    if (raw_data_[(size_t) Field::SOF] != 0x68 ||
        raw_data_[(size_t) Field::T] != 0x05 ||
        raw_data_[(size_t) Field::D] != 0x00 ||
        raw_data_.back() != 0xFE) {
        LOG(ERROR) << "包头不正确！";
        return false;
    }

    // 校验长度字段正确性
    int len = raw_data_[(size_t) Field::L];
    if (len != raw_data_.size() - 6) {
        LOG(ERROR) << "长度不正确！";
        return false;
    }

    // 校验码
    int sum = 0;
    for (int i = 0; i < raw_data_.size() - 2; ++i) {
        sum += raw_data_[i];
    }
    if ((int) (sum & 0xFF) != (int) raw_data_[raw_data_.size() - 2]) {
        LOG(ERROR) << "校验不正确！" << (int) (sum & 0xFF) << " " << (int) raw_data_[raw_data_.size() - 2];
        return false;
    }

    seq_ = raw_data_[(size_t) Field::A];
    command_ = raw_data_[(size_t) Field::C];
//    LOG(INFO) << (int) command;

    data_.clear();
    for (int i = (size_t) Field::DATA; i < raw_data_.size() - 2; ++i) {
        data_.push_back(raw_data_[i]);
    }

    return true;
}

void CommMsg::setErrorCode(uint16_t mask) {
    is_error_ = true;
    data_.resize(3);
    data_[0] = 0xFF;

    data_[1] = (mask >> 8) & 0xFF;
    data_[2] = (mask >> 0) & 0xFF;
}

void CommMsg::setData(const std::vector<uint8_t> &data) {
    data_.resize(1);
    data_[0] = 0x00;
    data_.insert(data_.end(), data.begin(), data.end());
}
}