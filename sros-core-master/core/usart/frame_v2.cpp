//
// Created by lhx on 17-2-18.
//

#include "frame_v2.h"

namespace usart {

FrameV2::FrameV2()
        : state_(F_IDLE),
          recv_payload_length_(0) {

}

bool FrameV2::feed(uint8_t c) {

    switch (state_) {
        case F_IDLE: {
            if (c == SOF_CHAR_A) {
                recv_payload_.clear();
                recv_payload_length_ = 0;
                recv_checksum_ = 0;

                state_ = F_SOF;
            }
            break;
        }
        case F_SOF: {
            if (c == SOF_CHAR_B) {
                state_ = F_LENGTH_HI;
            } else {
                state_ = F_IDLE;
            }
            break;
        }
        case F_LENGTH_HI: {
            recv_payload_length_ = c << 8;

            state_ = F_LENGTH_LO;
            break;
        }
        case F_LENGTH_LO: {
            recv_payload_length_ += c;
            remain_length_ = recv_payload_length_;

            state_ = F_PAYLOAD;
            break;
        }
        case F_PAYLOAD: {
            if (remain_length_ > 0) {
                recv_payload_.push_back(c);
                remain_length_ --;
            }
            if (remain_length_ == 0) {
                state_ = F_CKSUM_HI;
            }
            break;
        }
        case F_CKSUM_HI: {
            recv_checksum_ = c << 8;

            state_ = F_CKSUM_LO;
            break;
        }
        case F_CKSUM_LO: {
            recv_checksum_ += c;

            // 校验checksum
            if (recv_checksum_ == computeFrameChecksum(recv_payload_)) {
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

bool FrameV2::isFrameReady() const {
    return (state_ == F_READY);
}

const vector<uint8_t>& FrameV2::getPayload() {
    if (state_ == F_READY) {
        state_ = F_IDLE;
    }

    return recv_payload_;
}

uint16_t FrameV2::computeFrameChecksum(const vector<uint8_t> &payload) const {
    uint16_t checksum = 0;

    // TODO: use crc-16 checksum

    return checksum;
}

#define HI_BYTE_OF_WORD(x) ((uint8_t) ((x & 0xFF00) >> 8))
#define LO_BYTE_OF_WORD(x) ((uint8_t) (x & 0xFF))

vector<uint8_t> FrameV2::generateFrameData(const vector<uint8_t> &payload) const {
    vector<uint8_t> frame;

    uint16_t payload_len = (uint16_t) payload.size();

    uint16_t checksum = computeFrameChecksum(payload);

    frame.push_back(SOF_CHAR_A);
    frame.push_back(SOF_CHAR_B);

    frame.push_back(HI_BYTE_OF_WORD(payload_len));
    frame.push_back(LO_BYTE_OF_WORD(payload_len));

    for (auto it : payload) {
        frame.push_back(it);
    }

    frame.push_back(HI_BYTE_OF_WORD(checksum));
    frame.push_back(LO_BYTE_OF_WORD(checksum));

    return frame;
}

}
