/*
 * BaseMsg.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef BASEBAGMSG_H_
#define BASEBAGMSG_H_

#include <string>
#include <memory>

#include "core/util/time.h"

namespace sros {
namespace bag {

//enum TOPIC {
//    TOPIC_MAIN, TOPIC_X, TOPIC_Y,
////    TOPIC_EXIT
//};

enum MSG_TYPE {
    TYPE_TEST_STR,
    TYPE_LASER_SCAN_DATA,
    TYPE_POSE_STAMPED_DATA,
    TYPED_SLAM_COMMAND_DATA,
    TYPE_SLAM_STATE_DATA,
    TYPE_NETWORK_TASK,
    TYPE_COMMON_STATE,
    TYPE_COMMON_COMMAND,
    TYPE_OBSTACLE_DATA,
    TYPE_PATH_DATA,
    TYPE_DEBUG_CMD,
    TYPE_SET_PARAMETER,
    TYPE_USART_DATA,
    TYPE_SONAR_DATA,
    TYPE_SLAM_INFO_DATA,
};

const std::string TOPIC_X = "TOPIC_X";
const std::string TOPIC_Y = "TOPIC_Y";
const std::string TOPIC_MAIN = "TOPIC_MAIN";

typedef std::string topic_t;

class BaseMsg {
public:
    BaseMsg(topic_t topic, MSG_TYPE type) : topic_(topic),
                                            type_(type),
                                            is_real_time_(false) {
//    time_ = util::get_time_in_ms();
    }

    topic_t topic_;
    int64_t time_;

    MSG_TYPE type_;

    bool is_real_time_; // 是否为实时产生的msg,如果是,则只在队列中保留最新的msg

    virtual void getTime() = 0;

};


typedef std::shared_ptr<BaseMsg> base_msg_ptr;

} /* namespace core */
} /* namespace sros */


#endif /* BASEMSG_H_ */
