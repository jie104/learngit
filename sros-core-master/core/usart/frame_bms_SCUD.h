//
// Created by xuzheng on 7/19/17.
//

#ifndef SROS_FRAME_BATTERY_H
#define SROS_FRAME_BATTERY_H

#include <stdint.h>
#include <vector>

namespace usart {

using namespace std;

class FrameBMS_SCUD {  //数据帧
public:
    FrameBMS_SCUD();

    bool feed(uint8_t c);

    bool isFrameReady() const;

    const vector<uint8_t>& getPayload();

    vector<uint8_t> generateFrameData(const vector<uint8_t> &payload) const;

private:
    const int payload_length_ = 16; // payload length

    int remain_length_; // payload remain length to receive


    const uint8_t SOF_CHAR_1 = 0x96;
    const uint8_t SOF_CHAR_2 = 0xEB;
    const uint8_t SOF_CHAR_3 = 0x10;
    const uint8_t SOF_CHAR_4 = 0x05;
    vector<uint8_t> payload_;

    enum FrameState {
        F_IDLE,
        F_IDLE_READY_1,
        F_IDLE_READY_2,
        F_IDLE_READY_3,
        F_PAYLOAD,
        F_READY,
    };

    FrameState state_;
};

}

#endif //SROS_FRAME_BATTERY_H
