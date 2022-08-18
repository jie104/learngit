//
// Created by lhx on 17-2-18.
//

#ifndef SROS_FRAME_H
#define SROS_FRAME_H

#include <stdint.h>
#include <vector>

namespace usart {

using namespace std;

//#define PRINT_RAW_DATA  // 是否输出原始数据用于调试

#ifdef PRINT_RAW_DATA
#include <glog/logging.h>
#include "core/util/utils.h"

constexpr int MATCH_ID = 1;  // 输出匹配的ID,当构建Frame的模板参数和这个相对就输出日志
//#define CHECK_FRAME_INPUT    // 是否检测输入帧，检测输入帧就会将每次收到原始数据都打印
#endif

template <int ID = 0>
class FrameV1 {  //数据帧
 public:
    FrameV1() : state_(F_IDLE), payload_length_(0) {}

    bool feed(uint8_t c) {
        switch (state_) {
            case F_IDLE: {
#ifdef PRINT_RAW_DATA
                if (id_ == MATCH_ID) {
                    raw_data_.clear();
                }
#endif

                if (c == SOF_CHAR) {
                    payload_.clear();
                    payload_length_ = 0;
                    checksum_ = 0;

                    state_ = F_LENGTH_LO;
                }
                if (c == IAP_CHAR) {
                    payload_.clear();
                    // push IAP_CHAR int to IAP data
                    payload_.push_back(c);
                    payload_length_ = 0;
                    state_ = F_IAP_CMD;
                }
                break;
            }
            case F_IAP_CMD: {
                if (c == IAP_CONN || c == IAP_DISN || (c <= IAP_UTEST && c >= IAP_PACKET_INFO)) {
                    payload_.push_back(c);
                    state_ = F_IAP_LEN_LO;
                } else {
                    state_ = F_IDLE;
                }
                break;
            }
            case F_IAP_LEN_LO: {
                payload_length_ += c;
                payload_.push_back(c);

                state_ = F_IAP_LEN_HI;
                break;
            }
            case F_IAP_LEN_HI: {
                payload_length_ += (((uint16_t)c) << 8);
                remain_length_ = payload_length_;
                payload_.push_back(c);

                if (payload_length_ == 0) {
                    state_ = F_READY;
                } else if (payload_length_ < 200) {
                    state_ = F_IAP_PAYLOAD;
                } else {
                    state_ = F_IDLE;
                }
                break;
            }
            case F_LENGTH_LO: {
                payload_length_ += c;
                remain_length_ = payload_length_;

                state_ = F_PAYLOAD;
                break;
            }
            case F_IAP_PAYLOAD: {
                if (remain_length_ > 0) {
                    payload_.push_back(c);
                    remain_length_--;
                }
                if (remain_length_ == 0) {
                    state_ = F_READY;
                }
                break;
            }
            case F_PAYLOAD: {
                if (remain_length_ > 0) {
                    payload_.push_back(c);
                    remain_length_--;
                }
                if (remain_length_ == 0) {
                    state_ = F_CKSUM_LO;
                }
                break;
            }
            case F_CKSUM_LO: {
                checksum_ += c;

                state_ = F_EOF;
                break;
            }
            case F_EOF: {
                if (c != EOF_CHAR) {
                    // 帧尾错误，丢掉Frame，重新开始接收
                    state_ = F_IDLE;
                    break;
                }
                // 校验checksum
                if (checksum_ == computeFrameChecksum()) {
                    // Frame已经准备好
                    state_ = F_READY;
                } else {
                    // 校验失败，丢掉Frame，重新开始接收
                    state_ = F_IDLE;
                }
            }
            case F_READY: {
                break;
            }
            default: {
                break;
            }
        }
#ifdef PRINT_RAW_DATA
        if (id_ == MATCH_ID) {
            raw_data_.push_back(c);
#ifdef CHECK_FRAME_INPUT
            LOG(INFO) << "RX: " << raw_data_;
#else
            if (isFrameReady()) {
                LOG(INFO) << "RX: " << raw_data_;
            }
#endif
        }
#endif
        return isFrameReady();
    }

    bool isFrameReady() const { return (state_ == F_READY); }

    const vector<uint8_t>& getPayload() {
        if (state_ == F_READY) {
            state_ = F_IDLE;
        }

        return payload_;
    }

    uint16_t computeFrameChecksum() const {
        uint16_t checksum;

        checksum = 0;
        for (int i = 0; i < payload_length_; i++) {
            checksum ^= payload_[i];
        }

        return checksum;
    }

    vector<uint8_t> generateFrameData(const vector<uint8_t>& payload) const {
        if (payload[0] == IAP_CHAR) {
            return payload;
        }
        vector<uint8_t> frame;

        frame.push_back(SOF_CHAR);
        frame.push_back((uint8_t)payload.size());

        uint8_t checksum = 0;
        for (auto it : payload) {
            frame.push_back(it);
            checksum ^= it;  // 异或校验
        }

        frame.push_back(checksum);
        frame.push_back(EOF_CHAR);

#ifdef PRINT_RAW_DATA
        if (id_ == MATCH_ID) {
            LOG(INFO) << "TX: " << frame;
        }
#endif

        return frame;
    }

 private:
    uint16_t payload_length_;  // payload length
    uint16_t checksum_;

    uint16_t remain_length_;  // payload remain length to receive

    int id_ = ID;  // 标示是哪个的数据

#ifdef PRINT_RAW_DATA
    vector<uint8_t> raw_data_;
#endif

    const uint8_t SOF_CHAR = 0xAA;
    const uint8_t EOF_CHAR = 0x55;
    const uint8_t IAP_CHAR = 0xab;
    const uint8_t IAP_CONN = 0xfa;
    const uint8_t IAP_DISN = 0xaf;

    enum IAPCommnad {
        IAP_PACKET_INFO = 0xf1,
        IAP_PACKET_DATA,
        IAP_CHECK_RES,
        IAP_ACK,
        IAP_NACK,
        IAP_UTEST,
    };

    vector<uint8_t> payload_;

    enum FrameState {
        F_IDLE = 1,
        F_SOF = 2,
        F_LENGTH_HI = 3,
        F_LENGTH_LO = 4,
        F_PAYLOAD = 5,
        F_CKSUM_HI = 6,
        F_CKSUM_LO = 7,
        F_EOF = 8,
        F_READY = 9,
        F_IAP_START = 10,
        F_IAP_CMD = 11,
        F_IAP_LEN_HI = 12,
        F_IAP_LEN_LO = 13,
        F_IAP_PAYLOAD = 14,
        F_IAP_END = 15,
    };

    FrameState state_;
};

}  // namespace usart

#endif  // SROS_FRAME_H
