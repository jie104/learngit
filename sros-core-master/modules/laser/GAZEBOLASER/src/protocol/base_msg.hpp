//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_BASE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_BASE_MSG_H

#include "src_protocol.h"

#include <assert.h>
#include <cstring>
#include <memory>

namespace network {

const unsigned short ETHERNET_MAX_DATA_LENGTH = 1450; // 以太网上能够发送的最大单帧数据
//const unsigned short  MAX_BODY_LENGTH = MAX_DATA_LENGTH - SRC_HEADER_LENGTH;

// 各种Msg的父类，封装了公用函数
class BaseMsg {
public:
    BaseMsg(MSG_TYPE type)
            : type_(type),
              data_offset_(0) {
        switch (type) {
            case MSG_COMMAND :
            case MSG_STATE :
            case MSG_VELOCITY :
            case MSG_INFO:
            case MSG_POSE :
            case MSG_POSE_STAMPED: {
                MAX_DATA_LENGTH = 64;
                data_ = new uint8_t[64];
                break;
            }
            case MSG_GAZEBO_LASER_SCAN :
            case MSG_LASER_SCAN_STAMPED: {

                MAX_DATA_LENGTH = 1080 * 4 * 2 + 128;
                data_ = new uint8_t[MAX_DATA_LENGTH];
                break;
            }
            case PF_LASER_SCAN_STAMPED:{
                MAX_DATA_LENGTH = 3600 * 4 * 2 + 128;
                data_ = new uint8_t[MAX_DATA_LENGTH];
                break;
            }
            default: {
                MAX_DATA_LENGTH = ETHERNET_MAX_DATA_LENGTH;
                data_ = new uint8_t[MAX_DATA_LENGTH];
                break;
            }
        }
    };

    virtual ~BaseMsg() {
        if (data_) delete[] data_;
    }

    const unsigned int getHeaderLength() {
        return SRC_HEADER_LENGTH;
    }

    const unsigned int getBodyLength() {
        return length_ - SRC_HEADER_LENGTH;
    }

    void setBodyLength(const unsigned int length) {
        length_ = (unsigned short) (length + SRC_HEADER_LENGTH);
    }

    const unsigned short getLength() {
        return length_;
    }

    const MSG_TYPE &getType() const {
        return type_;
    }

    uint8_t *data() {
        return data_;
    }

    uint8_t *bodyData() {
        return data_ + SRC_HEADER_LENGTH;
    }

    bool encode() {
        data_offset_ = SRC_HEADER_LENGTH;
        // 获取body长度，必须先编码body
        if (encodeBody()) {
            // 编码完body后的偏移量就是整个data长度
            length_ = (unsigned short) data_offset_;
            assert(data_offset_ <= MAX_DATA_LENGTH);
            return encodeHeader();
        } else {
            return false;
        }
    }

    bool decode() {
        data_offset_ = SRC_HEADER_LENGTH;
        return decodeBody();
    }

    /// @brief msg data的前三个字节用于存储type和body长度
    bool encodeHeader() {
        data_[0] = (uint8_t) type_;

        unsigned short body_length = (unsigned short) (length_ - SRC_HEADER_LENGTH);
        data_[1] = (uint8_t) (body_length >> 8);
        data_[2] = (uint8_t) (body_length);
        return body_length > 0;
    }

    bool decodeHeader() {
        type_ = (MSG_TYPE) data_[0];
        length_ = (data_[1] << 8) + data_[2] + SRC_HEADER_LENGTH;
        return length_ > 0;
    }

    virtual bool encodeBody() = 0;

    virtual bool decodeBody() = 0;

protected:
    template<typename T>
    inline void encode_field(T field) {
        memcpy(data_ + data_offset_, &field, sizeof(T));
        data_offset_ += sizeof(T);
    }

    template<typename T>
    inline void encode_field(T *field, unsigned int size) {
        memcpy(data_ + data_offset_, field, size);
        data_offset_ += size;
    }

    template<typename T>
    inline void decode_field(T &field) {
        memcpy(&field, data_ + data_offset_, sizeof(T));
        data_offset_ += sizeof(T);
    }

    template<typename T>
    inline void decode_field(T *field, unsigned int size) {
        memcpy(field, data_ + data_offset_, size);
        data_offset_ += size;
    }

    MSG_TYPE type_;
//    uint8 data_[MAX_DATA_LENGTH]; // TODO 此处申请固定大小的空间有点浪费
    uint8_t *data_;

    unsigned int data_offset_;

    unsigned short length_;

    const unsigned short SRC_HEADER_LENGTH = 3;
    unsigned int MAX_DATA_LENGTH; // const
};

typedef std::shared_ptr<BaseMsg> BaseMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_BASE_MSG_H
