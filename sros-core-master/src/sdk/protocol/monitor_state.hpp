//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_MONITOR_STATE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_MONITOR_STATE_MSG_H

#include "base_msg.hpp"

namespace src {

/// @brief 监控SRC中重要变量
class MonitorStateMsg : public BaseMsg {
public:
    MonitorStateMsg() : BaseMsg(MSG_MONITOR_STATE){

    };

    virtual ~MonitorStateMsg() { };

    virtual bool encodeBody() override {
        encode_field(state_.value_0);
        encode_field(state_.value_1);
        encode_field(state_.value_2);
        encode_field(state_.value_3);
        encode_field(state_.value_4);
        encode_field(state_.value_5);

        return true;
    }

    virtual bool decodeBody() override {
        decode_field(state_.value_0);
        decode_field(state_.value_1);
        decode_field(state_.value_2);
        decode_field(state_.value_3);
        decode_field(state_.value_4);
        decode_field(state_.value_5);

        return true;
    }

    const void setSRCMonitorState(const SRCMonitorState& state) {
        state_ = state;
    }

    const SRCMonitorState& getSRCMonitorState() const {
        return state_;
    };

private:

    SRCMonitorState state_;
};

typedef std::shared_ptr<MonitorStateMsg> MonitorStateMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_MONITOR_STATE_MSG_H
