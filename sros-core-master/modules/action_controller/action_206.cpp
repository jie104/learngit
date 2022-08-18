//
// Created by caoyan on 7/20/21.
//

#include "action_206.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/msg/ObstacleMsg.hpp"

using namespace std;
using namespace sros::core;

namespace ac {

void Action206::doStart() {

    //所有206动作开启时都需要开启避障，动作结束时取消避障
    //搭桥、上下料时要忽略一部分蔽障
    is_ignore_left_obstacle_ = false;
    is_ignore_right_obstacle_ = false;

    if((action_param0_ == 2 && (action_param1_ == 1 || action_param1_ == 2 || action_param1_ == 5 || action_param1_ == 6))
        || action_param0_ == 11 || action_param0_ == 12 || action_param0_ == 21 || action_param0_ == 22) {
        is_ignore_left_obstacle_ = true;
    }

    if((action_param0_ == 2 && (action_param1_ == 3 || action_param1_ == 4 || action_param1_ == 7 || action_param1_ == 8))
        || action_param0_ == 13 || action_param0_ == 14 || action_param0_ == 23 || action_param0_ == 24) {
        is_ignore_right_obstacle_ = true;
    }

    LOG(INFO) << "action is_ignore_left_obstacle_: " << is_ignore_left_obstacle_
              << ", is_ignore_right_obstacle_: " << is_ignore_right_obstacle_;

    sendEacActionTask();

}

void Action206::onRegionObstacleMsg(const sros::core::base_msg_ptr &msg) {

    auto obstacle_msg = dynamic_pointer_cast<sros::core::ObstacleMsg>(msg);
    if(!obstacle_msg || obstacle_msg->oba_name.empty()) {
        return;
    }

    LOG(INFO) << "obstacle_msg->oba_name: " << obstacle_msg->oba_name
              << ", obstacle_msg->oba_state: " << obstacle_msg->oba_state;

    //环旭项目安全需求定制
    bool left_eu100_tim312_trigger = false;
    bool right_eu100_tim312_trigger = false;

    //左侧激光雷达
    if(obstacle_msg->oba_name == sros::device::DEVICE_UST_LIDAR_LEFT
       && obstacle_msg->oba_state == sros::core::ObstacleMsg::STATE_OBA_STOP_0
       && !is_ignore_left_obstacle_) {
        left_eu100_tim312_trigger = true;
        LOG(INFO) << "LEFT_UST_LIDAR_trigger";
    }

    //右侧激光雷达
    if(obstacle_msg->oba_name == sros::device::DEVICE_UST_LIDAR_RIGHT
       && obstacle_msg->oba_state == sros::core::ObstacleMsg::STATE_OBA_STOP_0
       && !is_ignore_right_obstacle_) {
        right_eu100_tim312_trigger = true;
        LOG(INFO) << "RIGHT_UST_LIDAR_trigger";
    }

    if(left_eu100_tim312_trigger || right_eu100_tim312_trigger) {
        LOG(INFO) << "trigger emergency";
        static std::string msg_source = "ActionController";
        auto d_msg = std::make_shared<sros::core::CommandMsg>(msg_source);
        d_msg->command = sros::core::CMD_TRIGGER_EMERGENCY;
        sros::core::MsgBus::sendMsg(d_msg);
    }

}

void Action206::onSrcAcFinishSucceed(int result_value) {
    //上传pgv扫码的值
    if(action_param0_ == 3 && (action_param1_ == 1 || action_param1_ == 2)) {
        action_task_->setResultValueStr(std::to_string(result_value));
    }

    doActionFinishSucceed(result_value);
}

}
