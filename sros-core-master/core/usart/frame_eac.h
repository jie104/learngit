//
// Created by john on 19-5-14.
//

#ifndef SROS_FRAME_EAC_H
#define SROS_FRAME_EAC_H

#include <stdint.h>
#include <vector>

namespace usart {

class FrameEAC {
public:
    FrameEAC() = default;

    bool feed(uint8_t c);

    bool isFrameReady() const;

    const std::vector<uint8_t> &getPayload();

    std::vector<uint8_t> generateFrameData(const std::vector<uint8_t> &payload) const;

private:
    const uint8_t RECIVER_START = 0xFF;

    std::vector<uint8_t> payload_;

    enum FrameState {
        F_IDLE,
        F_DATA,
        F_CKSUM,
        F_READY,
    };

    FrameState state_ = F_IDLE;
};

}


#endif //SROS_FRAME_EAC_H
