//
// Created by lhx on 17-2-18.
//

#ifndef SROS_FRAMEV2_H
#define SROS_FRAMEV2_H

#include <stdint.h>
#include <vector>

namespace usart {

using namespace std;

class FrameV2 {
public:
    FrameV2();
    
    bool feed(uint8_t c);

    bool isFrameReady() const;

    const vector<uint8_t>& getPayload();

    uint16_t computeFrameChecksum(const vector<uint8_t> &payload) const;

    vector<uint8_t> generateFrameData(const vector<uint8_t> &payload) const;

private:
    uint16_t recv_payload_length_; // payload length
    uint16_t recv_checksum_;

    uint16_t remain_length_; // payload remain length to receive

    const uint8_t SOF_CHAR_A = 0x55;
    const uint8_t SOF_CHAR_B = 0xAA;

    vector<uint8_t> recv_payload_;

    enum FrameState {
        F_IDLE,
        F_SOF,
        F_LENGTH_HI,
        F_LENGTH_LO,
        F_PAYLOAD,
        F_CKSUM_HI,
        F_CKSUM_LO,
        F_READY,
    };

    FrameState state_;
};

}


#endif //SROS_FRAMEV2_H
