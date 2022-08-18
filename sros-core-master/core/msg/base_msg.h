/*
 * BaseMsg.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef BASEMSG_H_
#define BASEMSG_H_

#include <string>
#include <memory>

#include "../util/time.h"

#include "../state.h"

namespace sros {
namespace core {

//enum TOPIC {
//    TOPIC_MAIN, TOPIC_X, TOPIC_Y,
////    TOPIC_EXIT
//};

enum MSG_TYPE {
    TYPE_TEST_STR,
    TYPE_LASER_SCAN_DATA,
    TYPE_POSE_STAMPED_DATA,
    TYPE_POSE_ODO_DATA,
    TYPED_SLAM_COMMAND_DATA,
    TYPE_SLAM_STATE_DATA,
    TYPE_NETWORK_TASK,
    TYPE_COMMON_STATE,
    TYPE_COMMON_COMMAND,
    TYPE_OBSTACLE_DATA,
    TYPE_AVOID_OBSTACLE_DATA,
    TYPE_PATH_DATA,
    TYPE_COMMAND,
    TYPE_SET_PARAMETER,
    TYPE_USART_DATA,
    TYPE_CAN_DATA,
    TYPE_SONAR_DATA,
    TYPE_COMMAND_RESPONSE,
    TYPE_COMMON,
    TYPE_IMAGE,
    TYPE_HUAWEI_COMM_DATA,
    TYPE_MODBUS_REGISTER,
    TYPE_LMKMATCH_DATA,
    TYPE_DUAL_SCAN_DATA,
    TYPE_DMCODE_DATA,
    TYPE_EMERGENCY_STATE,
    TYPE_HMI,
    TYPE_FEATURE_INFO,
    TYPE_IMU_MSG,
    TYPE_LIVOX_POINTS,
};

const std::string TOPIC_X = "TOPIC_X";
const std::string TOPIC_Y = "TOPIC_Y";
const std::string TOPIC_MAIN = "TOPIC_MAIN";

typedef std::string topic_t;

class BaseMsg {
public:
    BaseMsg(topic_t, MSG_TYPE type);
    BaseMsg(topic_t topic, MSG_TYPE type, bool is_real_time);

    topic_t topic_;
    int64_t time_;

    MSG_TYPE type_;

    bool is_real_time_; // 是否为实时产生的msg,如果是,则只在队列中保留最新的msg

    virtual void getTime() = 0;

};

inline BaseMsg::BaseMsg(topic_t topic, MSG_TYPE type)
        : topic_(topic),
          type_(type),
          is_real_time_(false) {
//    time_ = util::get_time_in_ms();
}

inline BaseMsg::BaseMsg(topic_t topic, MSG_TYPE type, bool is_real_time)
        : topic_(topic),
          type_(type),
          is_real_time_(is_real_time) {
//    time_ = util::get_time_in_ms();
}

typedef std::shared_ptr<BaseMsg> base_msg_ptr;

} /* namespace core */
} /* namespace sros */


#endif /* BASEMSG_H_ */
