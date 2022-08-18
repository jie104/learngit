//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_STATE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_STATE_MSG_H

#include "base_msg.hpp"

namespace src {

/// @brief SRC反馈的状态信息
///
/// 包括速度、加速度、正在执行路径编号等，但不包括POSE，
/// 因为Pose的上传频率较高，故Pose在PoseMsg中单独上传
class StateMsg : public BaseMsg {
public:
    StateMsg() : BaseMsg(MSG_STATE){
        state_.src_state = STATE_NONE;
        state_.movement_state = MOVEMENT_STILL;
    };

    virtual ~StateMsg() { };

    virtual bool encodeBody() override {
        encode_field(&state_.src_state, SRC_STATE_FIELD_SIZE);

        encode_field(state_.v_x);
        encode_field(state_.v_y);
        encode_field(state_.w);

        encode_field(state_.path_no);
        encode_field(state_.path_remain_time);
        encode_field(state_.path_remain_distance);
        encode_field(state_.path_total_distance);

        encode_field(&state_.movement_state, SRC_STATE_FIELD_SIZE);

        encode_field(state_.gpio_input);
        encode_field(state_.gpio_output);

        encode_field(&state_.error_reason, SRC_STATE_FIELD_SIZE);

        encode_field(state_.reserved_0);
        encode_field(state_.reserved_1);
        encode_field(state_.reserved_2);
        encode_field(state_.reserved_3);
        encode_field(state_.reserved_4);
        encode_field(state_.reserved_5);

        return true;
    }

    virtual bool decodeBody() override {
        decode_field(&state_.src_state, SRC_STATE_FIELD_SIZE);

        decode_field(state_.v_x);
        decode_field(state_.v_y);
        decode_field(state_.w);

        decode_field(state_.path_no);
        decode_field(state_.path_remain_time);
        decode_field(state_.path_remain_distance);
        decode_field(state_.path_total_distance);

        decode_field(&state_.movement_state, SRC_STATE_FIELD_SIZE);

        decode_field(state_.gpio_input);
        decode_field(state_.gpio_output);

        decode_field(&state_.error_reason, SRC_STATE_FIELD_SIZE);

        decode_field(state_.reserved_0);
        decode_field(state_.reserved_1);
        decode_field(state_.reserved_2);
        decode_field(state_.reserved_3);
        decode_field(state_.reserved_4);
        decode_field(state_.reserved_5);

        return true;
    }

    const void setSRCState(const SRCState& state) {
        state_ = state;
    }

    const SRCState& getSRCState() const {
        return state_;
    };

private:

    SRCState state_;
};

typedef std::shared_ptr<StateMsg> StateMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_STATE_MSG_H
