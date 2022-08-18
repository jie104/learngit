//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_POSE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_POSE_MSG_H

#include "base_msg.hpp"

namespace network {

/// @brief SRC上传的姿态信息
class PoseMsg : public BaseMsg {
public:
    PoseMsg() : BaseMsg(MSG_POSE) {

    }

    virtual ~PoseMsg() {

    }

    virtual bool encodeBody() override {
        encode_field(x_);
        encode_field(y_);
        encode_field(z_);
        encode_field(yaw_);
        encode_field(pitch_);
        encode_field(roll_);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(x_);
        decode_field(y_);
        decode_field(z_);
        decode_field(yaw_);
        decode_field(pitch_);
        decode_field(roll_);
        return true;
    }

    float getX() const {
        return x_;
    }

    void setX(float x) {
        x_ = x;
    }

    float getY() const {
        return y_;
    }

    void setY(float y) {
        y_ = y;
    }

    float getZ() const {
        return z_;
    }

    void setZ(float z) {
        z_ = z;
    }

    float getYaw() const {
        return yaw_;
    }

    void setYaw(float yaw) {
        yaw_ = yaw;
    }

    float getPitch() const {
        return pitch_;
    }

    void setPitch(float pitch) {
        pitch_ = pitch;
    }

    float getRoll() const {
        return roll_;
    }

    void setRoll(float roll) {
        roll_ = roll;
    }

private:
    float x_;
    float y_;
    float z_;

    float yaw_;
    float pitch_;
    float roll_;
};

typedef std::shared_ptr<PoseMsg> PoseMsg_ptr;

} /* namespace src */

#endif //SRC_SDK_NETWORK_PROTOCOL_POSE_MSG_H
