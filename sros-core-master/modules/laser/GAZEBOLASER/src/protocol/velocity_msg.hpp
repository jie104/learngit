//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_VELOCITY_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_VELOCITY_MSG_H

#include "base_msg.hpp"

namespace network {

// 在VELOCITY_MODE下，SRC需要执行的速度（v、w）
class VelocityMsg : public BaseMsg {
public:
    VelocityMsg() : BaseMsg(MSG_VELOCITY) {

    }

    virtual ~VelocityMsg() {

    }

    virtual bool encodeBody() override {
        encode_field(v_);
        encode_field(w_);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(v_);
        decode_field(w_);
        return true;
    }

    float getV() const {
        return v_;
    }

    void setV(float v) {
        v_ = v;
    }

    float getW() const {
        return w_;
    }

    void setW(float w) {
        w_ = w;
    }

private:
    float v_; // 线速度 m/s
    float w_; // 角速度 rad/s
};

typedef std::shared_ptr<VelocityMsg> VelocityMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_VELOCITY_MSG_H
