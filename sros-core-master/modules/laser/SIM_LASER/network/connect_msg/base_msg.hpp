//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_CONNECT_BASE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_CONNECT_BASE_MSG_H


#include <assert.h>
#include <cstring>
#include <memory>
#include <vector>
namespace connection {


const unsigned int MAX_DATA_LENGTH = 12600 * 8 + 128; // 最大单帧数据

// 各种Msg的父类，封装了公用函数
enum ConnectMsgType{
    BASE_MSG_TYPE = 1,
    SCAN_MSG_TYPE = 2,
};
class BaseMsg {
public:
    BaseMsg(ConnectMsgType type = BASE_MSG_TYPE)
            : type_(type),
              data_offset_(0) {
        data_.reserve(MAX_DATA_LENGTH);

        header_length = 1 + sizeof(length);
    };

    virtual ~BaseMsg() {

    }

    const ConnectMsgType &getType() const {
        return type_;
    }

    std::vector<uint8_t>& rawData(){
        return data_;
    }

    void rawData(const std::vector<uint8_t>& data) {
        data_ = data;
    }

    bool encode() {
        data_.clear();

        data_.push_back((uint8_t) (type_ + 0));
        data_offset_ = 1;
        encode_field(length);
        encodeBody();
        length = data_.size();
        memcpy(data_.data() + 1, &length, sizeof(length));
        return true;
    }

    bool decode() {
        type_ = (ConnectMsgType) data_[0];
        data_offset_ = 1;
        decode_field(length);
        decodeBody();
        return true;
    }

    unsigned int getHeaderLength() {
        return header_length;
    }

    unsigned int getBodyLength() {
        return length - header_length;
    }

    unsigned int getLength() {
        return length;
    }
protected:

    virtual bool encodeBody() {
        return false;
    };

    virtual bool decodeBody() {
        return false;
    };

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

    ConnectMsgType type_;
    std::vector<uint8_t> data_;

    unsigned int data_offset_;

    unsigned int header_length; // const

    unsigned int length;
};

typedef std::shared_ptr<BaseMsg> BaseMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_UART_BASE_MSG_H
