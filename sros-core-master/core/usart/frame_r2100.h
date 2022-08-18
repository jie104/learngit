//
// Created by xuzheng on 17-7-12.
//

#ifndef SROS_FRAME_R2100_H
#define SROS_FRAME_R2100_H

#include <stdint.h>
#include <vector>

namespace usart {

using namespace std;
class FrameR2100 {  //数据帧
public:
    FrameR2100();

    bool feed(uint8_t c);

    bool isFrameReady() const;

    const vector<uint8_t>& getPayload();

    uint16_t computeFrameChecksum() const;

    vector<uint8_t> generateFrameData(const vector<uint8_t> &payload) const;

private:
    uint16_t payload_length_; // payload length
    uint16_t checksum_;

    uint16_t remain_length_; // payload remain length to receive

    const uint8_t RECEIVER_ID = 0xde;
    const uint8_t SENDER_ID = 0x01;
    const uint8_t RECV_CMD = 0x11;

    vector<uint8_t> payload_;

    enum FrameState {
        F_IDLE,
        F_IDLE_READY,
        F_LENGTH_LO,
        F_RECV_CMD,
        F_PAYLOAD,
        F_CKSUM_LO,
        F_READY,
    };

    FrameState state_;
};

}
#endif //SROS_FRAME_R2100_H
