//
// Created by xuzheng on 7/19/17.
//

#include "frame_bms_SCUD.h"
#include "core/core.h"

namespace usart {

FrameBMS_SCUD::FrameBMS_SCUD()
        : state_(F_IDLE),
          remain_length_(0) {

}

bool FrameBMS_SCUD::feed(uint8_t c) {

    switch (state_) {
        case F_IDLE: {
            if (c == SOF_CHAR_1) {
                payload_.clear();
                payload_.push_back(c);
                remain_length_ = payload_length_;
                remain_length_ --;
                state_ = F_IDLE_READY_1;
            }
            break;
        }
        case F_IDLE_READY_1: {
            if (c == SOF_CHAR_2) {
                payload_.push_back(c);
                remain_length_ --;
                state_ = F_IDLE_READY_2;
            } else{
                state_ = F_IDLE;
            }
            break;
        }
        case F_IDLE_READY_2: {
            if (c == SOF_CHAR_3) {
                payload_.push_back(c);
                remain_length_ --;
                state_ = F_IDLE_READY_3;
            } else{
                state_ = F_IDLE;
            }
            break;
        }
        case F_IDLE_READY_3: {
            if (c == SOF_CHAR_4) {
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

bool FrameBMS_SCUD::isFrameReady() const {
    return (state_ == F_READY);
}

const vector<uint8_t> &FrameBMS_SCUD::getPayload() {
    if (state_ == F_READY) {
        state_ = F_IDLE;
    }

    return payload_;
}

vector<uint8_t> FrameBMS_SCUD::generateFrameData(const vector<uint8_t> &payload) const {
    //我们不适用此函数，只是为了能FrameBattery在文件connection.hpp通过
    vector<uint8_t> frame;
    return frame;
}

}
