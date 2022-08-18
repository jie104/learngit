//
// Created by yangjia on 18-5-21.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_VELOCITY_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_VELOCITY_MSG_H


#include "base_msg.hpp"

namespace src {

/// @brief SRC上传的姿态信息
class VelocityMsg : public BaseMsg {
public:
    VelocityMsg() : BaseMsg(MSG_VELOCITY) {

    }

    virtual ~VelocityMsg() {

    }


    virtual bool encodeBody() override {
        encode_field(timestamp_);
        encode_field(vx_);
        encode_field(vy_);
        encode_field(vtheta_);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(timestamp_);
        decode_field(vx_);
        decode_field(vy_);
        decode_field(vtheta_);
        return true;
    }


    uint32_t getTimestamp() const {
        return timestamp_;
    }

    void setTimestamp(const uint32_t &timestamp) {
        timestamp_ = timestamp;
    }

    int32_t getVx() const {
        return vx_;
    }

    void setVx(int32_t Vx) {
        vx_ = Vx;
    }


    int32_t getVy() const {
        return vy_;
    }

    void setVy(int32_t Vy) {
        vy_ = Vy;
    }

    int32_t getVtheta() const {
        return vtheta_;
    }

    void setVtheta(int32_t Vtheta) {
       vtheta_ = Vtheta;
    }




private:
    uint32_t timestamp_; // ms
    int32_t vx_;
    int32_t vy_;
    int32_t vtheta_;


};

typedef std::shared_ptr<VelocityMsg> VelocityMsg_ptr;

} /* namespace src */


#endif //SROS_VELOCITY_MSG_HPP
