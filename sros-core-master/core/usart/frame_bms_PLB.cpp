//
// Created by lhx on 9/21/17.
//

#include "frame_bms_PLB.h"
#include "core/core.h"

namespace usart {

FrameBMS_PLB::FrameBMS_PLB()
        : state_(F_IDLE),
          remain_length_(0) {

}

bool FrameBMS_PLB::feed(uint8_t c) {
    switch (state_) {
        case F_IDLE: {
            if (c == SOF_CHAR_1) {
                state_ = F_IDLE_READY_1;
            }
            break;
        }
        case F_IDLE_READY_1: {
            if (c == SOF_CHAR_2) {
                state_ = F_IDLE_READY_2;
            } else {
                state_ = F_IDLE;
            }
            break;
        }
        case F_IDLE_READY_2: {
            if (c == SOF_CHAR_3) {
                state_ = F_IDLE_READY_3;
            } else{
                state_ = F_IDLE;
            }
            break;
        }
        case F_IDLE_READY_3: {
            if (c == SOF_CHAR_4) {
                remain_length_ = payload_length_;
                payload_.clear();
                state_ = F_PAYLOAD;
            } else{
                state_ = F_IDLE;
            }
            break;
        }
        case F_PAYLOAD: {
            if (remain_length_ > 0) {
                payload_.push_back(c);
                remain_length_--;
            }
            if (remain_length_ == 0) {
                state_ = F_READY;
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

bool FrameBMS_PLB::isFrameReady() const {
    return (state_ == F_READY);
}

const vector<uint8_t> &FrameBMS_PLB::getPayload() {
    if (state_ == F_READY) {
        state_ = F_IDLE;
    }

    return payload_;
}

vector<uint8_t> FrameBMS_PLB::generateFrameData(const vector<uint8_t> &payload) const {
    // 力朗电池查询指令
    // DD A5 03 00 FF FD 77
    vector<uint8_t> frame;
    frame.push_back(0xDD);
    frame.push_back(0xA5);
    frame.push_back(0x03);
    frame.push_back(0x00);
    frame.push_back(0xFF);
    frame.push_back(0xFD);
    frame.push_back(0x77);
    return frame;
}

}
