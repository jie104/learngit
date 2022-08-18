//
// Created by xuzheng on 17-7-12.
//

#include "frame_r2100.h"
namespace usart {

FrameR2100::FrameR2100()
        : state_(F_IDLE),
          payload_length_(0) {

}

bool FrameR2100::feed(uint8_t c) {


    switch (state_) {
        case F_IDLE: {
            if (c == SENDER_ID) {
                payload_.clear();
                payload_length_ = 0;
                checksum_ = 0;

                payload_.push_back(c);

                state_ = F_IDLE_READY;
            }
            break;
        }
        case F_IDLE_READY: {
            if (c == RECEIVER_ID) {
                payload_.push_back(c);

                state_ = F_LENGTH_LO;
            } else{
                state_ = F_IDLE;
            }
            break;
        }
        case F_LENGTH_LO: {
            payload_.push_back(c);
            payload_length_ = c;
            payload_length_ -= 1; //除了校验和，其余的都接收
            remain_length_ = payload_length_;

            remain_length_ -= 3;

            state_ = F_RECV_CMD;
            break;
        }
        case F_RECV_CMD: {

            if(c == RECV_CMD){
                payload_.push_back(c);
                remain_length_ --;
                state_ = F_PAYLOAD;
            } else{
                state_ = F_IDLE;
            }
            break;
        }
        case F_PAYLOAD: {
            if (remain_length_ > 0) {
                payload_.push_back(c);
                remain_length_ --;
            }
            if (remain_length_ == 0) {
                state_ = F_CKSUM_LO;
            }
            break;
        }
        case F_CKSUM_LO: {
            checksum_ += c;
            // 校验checksum
            if (checksum_ == computeFrameChecksum()) {
                // Frame已经准备好
                state_ = F_READY;

            } else {
                // 校验失败，丢掉Frame，重新开始接收
                state_ = F_IDLE;
            }
            break;
        }
        case F_READY: {
            break;
        }
        default: {
            break;
        }
    }

    return isFrameReady();
}

bool FrameR2100::isFrameReady() const {
    return (state_ == F_READY);
}

const vector<uint8_t>& FrameR2100::getPayload() {
    if (state_ == F_READY) {
        state_ = F_IDLE;
    }

    return payload_;
}

uint16_t FrameR2100::computeFrameChecksum() const {
    uint16_t checksum;

    checksum = 0;
    for (int i = 0; i < payload_length_; i++) {
        checksum ^= payload_[i];
    }

    return checksum;
}

vector<uint8_t> FrameR2100::generateFrameData(const vector<uint8_t> &payload) const {
    vector<uint8_t> frame;

    frame.push_back(RECEIVER_ID);
    frame.push_back(SENDER_ID);
    frame.push_back((uint8_t) 0x05); //length of the complete frame
    for (auto it : payload) {
        frame.push_back(it);
    }
    uint8_t checksum = 0;
    for (auto it : frame) {
        checksum ^= it; // 异或校验
    }
    frame.push_back(checksum); //checksum
    return frame;
}

}
