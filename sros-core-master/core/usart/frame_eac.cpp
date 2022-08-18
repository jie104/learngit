//
// Created by john on 19-5-14.
//

#include "frame_eac.h"

namespace usart {

bool FrameEAC::feed(uint8_t c) {
    switch (state_) {
        case F_IDLE: {
            if (c == RECIVER_START) {
                payload_.clear();

                state_ = F_DATA;
            }
            break;
        }
        case F_DATA: {
            payload_.push_back(c);
            if (payload_.size() == 8) {
                state_ = F_CKSUM;
            }
            break;
        }
        case F_CKSUM: {
            uint8_t sum = 0xff;
            for (auto data : payload_) {
                sum += data;
            }
            if (sum == c) {
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
    return false;
}

bool FrameEAC::isFrameReady() const {
    return (state_ == F_READY);
}

const std::vector<uint8_t> &FrameEAC::getPayload() {
    if (state_ == F_READY) {
        state_ = F_IDLE;
    }

    return payload_;
}

std::vector<uint8_t> FrameEAC::generateFrameData(const std::vector<uint8_t> &payload) const {
    std::vector<uint8_t> frame;

    frame.push_back(RECIVER_START);
    for (auto it : payload) {
        frame.push_back(it);
    }
    uint8_t checksum = 0xff;
    for (auto it : payload) {
        checksum += it;
    }
    frame.push_back(checksum); //checksum
    return frame;
}

}