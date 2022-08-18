//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_UART_BASE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_UART_BASE_MSG_H

#include "src_protocol.h"

#include <assert.h>
#include <cstring>
#include <memory>

namespace src {

using namespace std;

const unsigned short UART_MAX_DATA_LENGTH = 256; // UART能够发送的最大单帧数据

// 各种Msg的父类，封装了公用函数
class BaseMsg {
public:
    BaseMsg(MSG_TYPE type)
            : type_(type),
              data_offset_(0) {
        data_.reserve(UART_MAX_DATA_LENGTH);
    };

    virtual ~BaseMsg() {

    }

    const MSG_TYPE &getType() const {
        return type_;
    }

    const vector<uint8_t>& rawData() const {
        return data_;
    }

    void rawData(const vector<uint8_t>& data) {
        data_ = data;
    }

    bool encode() {
        data_.clear();

        data_.push_back((uint8_t) (type_ + 0));
        data_offset_ = 1;

        return encodeBody();
    }

    bool decode() {
        type_ = (MSG_TYPE_t) data_[0];
        data_offset_ = 1;

        return decodeBody();
    }

    virtual bool encodeBody() = 0;

    virtual bool decodeBody() = 0;

protected:
    template<typename T>
    inline void encode_field(T field) {
        data_.resize(data_.size() + sizeof(T));
        memcpy(data_.data() + data_offset_, &field, sizeof(T));
        data_offset_ += sizeof(T);
    }

    template<typename T>
    inline void encode_field(T *field, unsigned int size) {
        data_.resize(data_.size() + size);
        memcpy(data_.data() + data_offset_, field, size);
        data_offset_ += size;
    }

    template<typename T>
    inline void decode_field(T &field) {
        memcpy(&field, data_.data() + data_offset_, sizeof(T));
        data_offset_ += sizeof(T);
    }

    template<typename T>
    inline void decode_field(T *field, unsigned int size) {
        memcpy(field, data_.data() + data_offset_, size);
        data_offset_ += size;
    }

    MSG_TYPE type_;
    vector<uint8_t> data_;

    unsigned int data_offset_;

    unsigned int MAX_DATA_LENGTH; // const
};

typedef std::shared_ptr<BaseMsg> BaseMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_UART_BASE_MSG_H
