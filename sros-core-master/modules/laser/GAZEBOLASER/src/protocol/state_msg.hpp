//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_STATE_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_STATE_MSG_H

#include "base_msg.hpp"

namespace network {

/// @brief SRC反馈的状态信息
///
/// 包括速度、加速度、正在执行路径编号等，但不包括POSE，
/// 因为Pose的上传频率较高，故Pose在PoseMsg中单独上传
class StateMsg : public BaseMsg {
public:
    StateMsg() : BaseMsg(MSG_STATE), src_state_(STATE_NONE) { };

    virtual ~StateMsg() { };

    virtual bool encodeBody() override {
        encode_field(&src_state_, SRC_STATE_FIELD_SIZE);
        encode_field(path_no_);
        encode_field(v_);
        encode_field(w_);
        encode_field(remain_time_);
        encode_field(remain_distance_);
        encode_field(total_distance_);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(&src_state_, SRC_STATE_FIELD_SIZE);
        decode_field(path_no_);
        decode_field(v_);
        decode_field(w_);
        decode_field(remain_time_);
        decode_field(remain_distance_);
        decode_field(total_distance_);
        return true;
    }

    const SRC_STATE_t &getSrc_state() const {
        return src_state_;
    }

    float getV() const {
        return v_;
    }

    float getW() const {
        return w_;
    }

    int16_t getRemainTime() const {
        return remain_time_;
    }

    int16_t getRemainDistance() const {
        return remain_distance_;
    }

    int16_t getTotalDistance() const {
        return total_distance_;
    }

    void setRemainInfo(int16_t remain_time, int16_t remain_distance, int16_t total_distance) {
        remain_time_ = remain_time;
        remain_distance_ = remain_distance;
        total_distance_ = total_distance;
    }

    uint8_t getPath_no() const {
        return path_no_;
    }

    void setState(SRC_STATE_t src_state, uint8_t path_no) {
        src_state_ = src_state;
        path_no_ = path_no;
    }

    void setVelocity(float v, float w) {
        v_ = v;
        w_ = w;
    }

private:
    SRC_STATE_t src_state_; // SRC运行状态
    float v_; // 当前线速度
    float w_; // 当前角速度
    uint8_t path_no_; // 当前执行的path编号

    int16_t remain_time_; // 当前路径剩余执行时间(s)
    int16_t remain_distance_; // 当前路径剩余长度(cm)
    int16_t total_distance_; // 当前路径总距离(cm)
};

typedef std::shared_ptr<StateMsg> StateMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_STATE_MSG_H
