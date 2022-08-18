//
// Created by lhx on 9/21/17.
//

#ifndef SROS_FRAME_BMS_PLB_H
#define SROS_FRAME_BMS_PLB_H

#include <stdint.h>
#include <vector>

namespace usart {

using namespace std;

// 定制力朗锂电池BMS通信协议
class FrameBMS_PLB {  //数据帧
public:
    FrameBMS_PLB();

    bool feed(uint8_t c);

    bool isFrameReady() const;

    const vector<uint8_t>& getPayload();

    vector<uint8_t> generateFrameData(const vector<uint8_t> &payload) const;

private:
    const int payload_length_ = 30; // payload length

    int remain_length_; // payload remain length to receive

    const uint8_t SOF_CHAR_1 = 0xDD;
    const uint8_t SOF_CHAR_2 = 0xA5;
    const uint8_t SOF_CHAR_3 = 0x00;
    const uint8_t SOF_CHAR_4 = 0x1B;
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

#endif //SROS_FRAME_BMS_PLB_H
