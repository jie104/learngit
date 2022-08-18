//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_OPT_POSE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_OPT_POSE_MSG_H

#include "base_msg.hpp"

namespace src {

#define FLOAT2INT32_M(x) ((int32_t) (x * 1000))
#define FLOAT2INT32_RAD(x) ((int32_t) (x * 10000))

/// @brief SRC上传的姿态信息
class OptPoseMsg : public BaseMsg {
public:
    OptPoseMsg() : BaseMsg(MSG_OPT_POSE) {

    }

    virtual ~OptPoseMsg() {

    }

    virtual bool encodeBody() override {
        encode_field(timestamp_);

        encode_field(x_);
        encode_field(y_);
        encode_field(z_);
        encode_field(yaw_);
        encode_field(pitch_);
        encode_field(roll_);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(timestamp_);

        decode_field(x_);
        decode_field(y_);
        decode_field(z_);
        decode_field(yaw_);
        decode_field(pitch_);
        decode_field(roll_);
        return true;
    }

    uint64_t getTimestamp() const {
        return timestamp_;
    }

    void setTimestamp(const uint64_t &timestamp) {
        timestamp_ = timestamp;
    }

    int32_t getX() const {
        return x_;
    }

    void setX(int32_t x) {
        x_ = x;
    }

    int32_t getY() const {
        return y_;
    }

    void setY(int32_t y) {
        y_ = y;
    }

    int32_t getZ() const {
        return z_;
    }

    void setZ(int32_t z) {
        z_ = z;
    }

    int16_t getYaw() const {
        return yaw_;
    }

    void setYaw(int16_t yaw) {
        yaw_ = yaw;
    }

    int16_t getPitch() const {
        return pitch_;
    }

    void setPitch(int16_t pitch) {
        pitch_ = pitch;
    }

    int16_t getRoll() const {
        return roll_;
    }

    void setRoll(int16_t roll) {
        roll_ = roll;
    }

private:
    uint32_t timestamp_; // ms

    int32_t x_; // mm
    int32_t y_;
    int32_t z_;

    int16_t yaw_; // (1/10000)rad
    int16_t pitch_;
    int16_t roll_;
};

typedef std::shared_ptr<OptPoseMsg> OptPoseMsg_ptr;

} /* namespace src */

#endif //SRC_SDK_NETWORK_PROTOCOL_POSE_MSG_H
