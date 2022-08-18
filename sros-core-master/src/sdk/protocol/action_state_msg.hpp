//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_ACTION_STATE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_ACTION_STATE_MSG_H

#include "base_msg.hpp"

namespace src {

/// @brief SRC反馈的动作状态
class ActionStateMsg : public BaseMsg {
public:
    ActionStateMsg() : BaseMsg(MSG_ACTION_STATE){

    };

    virtual ~ActionStateMsg() { };

    virtual bool encodeBody() override {
        encode_field(state_.action_no);
        encode_field(state_.action_result);
        encode_field(state_.action_result_value);
        encode_field(state_.action_reserved_0);
        encode_field(state_.action_reserved_1);

        return true;
    }

    virtual bool decodeBody() override {
        decode_field(state_.action_no);
        decode_field(state_.action_result);
        decode_field(state_.action_result_value);
        decode_field(state_.action_reserved_0);
        decode_field(state_.action_reserved_1);

        return true;
    }

    const void setSRCActionState(const SRCActionState& state) {
        state_ = state;
    }

    const SRCActionState& getSRCActionState() const {
        return state_;
    };

private:

    SRCActionState state_;
};

typedef std::shared_ptr<ActionStateMsg> ActionStateMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_ACTION_STATE_MSG_H
