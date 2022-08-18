//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_BASE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_BASE_MSG_H

#include <assert.h>
#include <cstring>
#include <memory>

namespace record {

const unsigned short ETHERNET_MAX_DATA_LENGTH = 1450; // 以太网上能够发送的最大单帧数据
//const unsigned short  MAX_BODY_LENGTH = MAX_DATA_LENGTH - SRC_HEADER_LENGTH;

typedef enum MSG_TYPE {
    MSG_COMMAND = 0x01,
    MSG_STATE,
    MSG_VELOCITY,
    MSG_POSE,
    MSG_PATH,
    MSG_GAZEBO_LASER_SCAN,
    MSG_PARAMETER,
    MSG_SIGNAL,
    MSG_INFO,
    MSG_LASER_SCAN_STAMPED,
    MSG_POSE_STAMPED,
    MSG_USART_DATA,
    PF_LASER_SCAN_STAMPED,
    MSG_GPS,
    UHD_LASER_SCAN_STAMPED,
    MSG_SLAM_INFO = 0x10,
} MSG_TYPE_t;

// 各种Msg的父类，封装了公用函数
class BaseMsg {
public:
    BaseMsg(MSG_TYPE type)
            : type_(type),
              data_offset_(0),data_(0),SRC_HEADER_LENGTH(3) {
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

            case UHD_LASER_SCAN_STAMPED: {
                MAX_DATA_LENGTH = 12600 * 4 * 2 + 128;
                data_ = new uint8_t[MAX_DATA_LENGTH];
                SRC_HEADER_LENGTH = 5;
                break;
            }

            case MSG_GPS: {
                MAX_DATA_LENGTH = 162 + 128;
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
        length_ = (unsigned int) (length + SRC_HEADER_LENGTH);
    }

    const unsigned int getLength() {
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
            length_ = (unsigned int) data_offset_;
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
        unsigned int body_length = (length_ - SRC_HEADER_LENGTH);
        if (SRC_HEADER_LENGTH == 3) {

            data_[1] = (uint8_t) (body_length >> 8);
            data_[2] = (uint8_t) (body_length);
        }else{
            data_[1] = (uint8_t) (body_length >> 24);
            data_[2] = (uint8_t) (body_length >> 16);
            data_[3] = (uint8_t) (body_length >> 8);
            data_[4] = (uint8_t) (body_length);
        }

        return body_length > 0;
    }

    bool decodeHeader() {
        if (SRC_HEADER_LENGTH == 3) {
            type_ = (MSG_TYPE) data_[0];
            length_ = (data_[1] << 8) + data_[2] + SRC_HEADER_LENGTH;
        }else{
            type_ = (MSG_TYPE) data_[0];
            length_ = (data_[1] << 24) + (data_[2] << 16) + (data_[3] << 8) + data_[4] + SRC_HEADER_LENGTH;
            if (length_ > 126000) {
                SRC_HEADER_LENGTH = 3;
                length_ = (data_[1] << 8) + data_[2] + SRC_HEADER_LENGTH;
            }
        }

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

    unsigned int length_;

    unsigned int SRC_HEADER_LENGTH;
    unsigned int MAX_DATA_LENGTH; // const
};

typedef std::shared_ptr<BaseMsg> BaseMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_BASE_MSG_H
