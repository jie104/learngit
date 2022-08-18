//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_SIGNAL_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_SIGNAL_MSG_H

#include "base_msg.hpp"

namespace network {

/// @brief SRC发送的信号
class SignalMsg : public BaseMsg {
public:
    SignalMsg() : BaseMsg(MSG_SIGNAL), signal_(SIGNAL_NONE) { };

    virtual ~SignalMsg() { };

    virtual bool encodeBody() override {
        encode_field(&signal_, SRC_SIGNAL_FIELD_SIZE);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(&signal_, SRC_SIGNAL_FIELD_SIZE);
        return true;
    }

    SRC_SIGNAL_t getSignal() const {
        return signal_;
    }

    void setSignal(SRC_SIGNAL_t signal) {
        signal_ = signal;
    }

private:
    SRC_SIGNAL_t signal_;

};

typedef std::shared_ptr<SignalMsg> SignalMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_SIGNAL_MSG_H
