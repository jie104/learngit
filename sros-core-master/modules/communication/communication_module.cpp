/**
 * @file communication_module.cpp
 *
 * @author pengjiali
 * @date 18-10-25.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include <core/state.h>
#include "communication_module.h"
#include "core/msg/PoseStampedMsg.h"
#include "core/msg/command_msg.hpp"
#include "core/task/task_manager.h"
#include "core/logger.h"
#include "core/settings.h"

using namespace std;

namespace sros {

CommunicationModule::CommunicationModule(const std::string &name) : Module(name) {}

CommunicationModule::~CommunicationModule() {}

void CommunicationModule::run() {
    if (!subClassRunPrepare()) {
        return;
    }

    subscribeTopic("TOPIC_CMD_RESPONSE", CALLBACK(&CommunicationModule::onCommandResponseMsg));
    subscribeTopic("TOPIC_NOTIFY", CALLBACK(&CommunicationModule::onNotifyMsg));

    dispatch();
}

bool CommunicationModule::moveToStation(core::TaskNo_t no, uint16_t station_id,
                                        core::ObstacleAvoidPolicy avoid_policy) {
    LOG(INFO) << "CommunicationModule::moveToStation(): " << station_id << " avoid_policy = " << avoid_policy;
    if (station_id == 0) {
        return false;
    }

    std::deque<sros::core::StationNo_t> dst_stations;
    dst_stations.push_back((sros::core::StationNo_t) station_id);

    auto new_task = std::make_shared<sros::core::MovementTask>(no, getName(), dst_stations, avoid_policy);
    new_task->setTaskSessionId(0);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
    mm->movement_task = new_task;
    mm->is_real_time_ = true;

    return sendCommandMsg(mm);
}

bool CommunicationModule::moveToPose(core::TaskNo_t no, const core::Pose &pose,
                                     core::ObstacleAvoidPolicy avoid_policy) {
    LOG(INFO) << "CommunicationModule::moveToPose(): " << pose << " avoidPolicy: " << avoid_policy;

    std::deque<sros::core::Pose> dst_poses;
    dst_poses.push_back(pose);

    auto new_task = std::make_shared<sros::core::MovementTask>(no, getName(), dst_poses, avoid_policy);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
    mm->movement_task = new_task;

    return sendCommandMsg(mm);
}

bool CommunicationModule::startNewAction(core::TaskNo_t no, int id, int param0, int param1) {
    auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();

    if (cur_task && cur_task->isRunning()) {
        LOG(WARNING) << "ACTION_TASK: previous task is running, new task is ignored.";
        return false;
    }

    return startNewCoexistAction(no, id, param0, param1);
}

bool CommunicationModule::startNewCoexistAction(core::TaskNo_t no, int id, int param0, int param1) {
    if (g_state.isEmergency()) {
        LOG(WARNING) << "ACTION_TASK: vehicle is in emergency mode!";
        return false;
    }

    auto new_task = std::make_shared<sros::core::ActionTask>(no, getName(), id, param0, param1);
    new_task->setTaskSeq(0);
    new_task->setTaskSessionId(0);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_NEW_ACTION_TASK;
    mm->param0 = new_task->getTaskNo();
    mm->action_task = new_task;

    sendMsg(mm);

    return true;
}

bool CommunicationModule::setCurMap(const string &map_name) {
    LOG(INFO) << "CommunicationModule::setCurMap() " << map_name;
    if (map_name.empty()) {
        return false;
    }

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_SET_CUR_MAP;
    mm->map_name = map_name;

    return sendCommandMsg(mm);
}

bool CommunicationModule::startMission(uint16_t mission_id) {
    LOGGER(INFO, MODBUS) << "Start task, task id: " << mission_id;

    auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
    d_msg->req_seq = 0;
    d_msg->session_id = 0;
    d_msg->command = sros::core::CMD_START_MISSION;
    d_msg->mission_no = sros::core::util::get_timestamp_in_ms();
    d_msg->mission_id = mission_id;
    d_msg->mission_cur_step_id = "";
    d_msg->mission_avoid_policy = sros::core::ObstacleAvoidPolicy::OBSTACLE_AVOID_WAIT;
    d_msg->user_name = "";

    return sendCommandMsg(d_msg);
}

bool CommunicationModule::pauseMission() {
    LOGGER(INFO, MODBUS) << "Pause task";
    return srcPause();
}

bool CommunicationModule::continueMission() {
    LOGGER(INFO, MODBUS) << "Continue task";

    auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
    d_msg->req_seq = 0;
    d_msg->session_id = 0;
    d_msg->command = sros::core::CMD_CONTINUE_MISSION;
    return sendCommandMsg(d_msg);

}

bool CommunicationModule::cancelMission(uint64_t uuid) {
    LOGGER(INFO, MODBUS) << "Cancel task: " << uuid;

    auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
    d_msg->req_seq = 0;
    d_msg->session_id = 0;
    d_msg->command = sros::core::CMD_CANCEL_MISSION;
    d_msg->mission_no = uuid;
    return sendCommandMsg(d_msg);
}

bool CommunicationModule::startLocationByStation(uint16_t station_id) {
    // 构造DebugCmdMsg处理
    return sendSimpleCommandMsg(sros::core::CMD_START_LOCATION, station_id);
}

bool CommunicationModule::startLocationByPose(const core::Pose &pose, bool absolute_location) {
    auto d_msg = make_shared<sros::core::CommandMsg>(getName());
    d_msg->pose = pose;
    d_msg->command = core::CMD_SET_LOCATION_INITIAL_POSE;

    if (!sendCommandMsg(d_msg)) {
        return false;
    }

    LOG(INFO) << "CommunicationModule: startLocationByPose (" << pose.x() << ", " << pose.y() << ", " << pose.yaw()
              << ")";

    d_msg = std::make_shared<sros::core::CommandMsg>(getName());
    d_msg->command = sros::core::CMD_START_LOCATION;
    d_msg->param0 = 0;
    d_msg->param_boolean = absolute_location;

    sendMsg(d_msg);
    return true;  // 定位的时间过长，直接回复成功
}

bool CommunicationModule::stopLocation() {
    LOG(INFO) << "CommunicationModule::stopLocation()";
    return sendSimpleCommandMsg(sros::core::CMD_STOP_LOCATION);
}

bool CommunicationModule::sendSimpleCommandMsg(const core::CommandType &command_type, int param0, int param1,
                                               int param2) {
    auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
    d_msg->command = command_type;
    d_msg->param0 = param0;
    d_msg->param1 = param1;
    d_msg->param2 = param2;

    return sendCommandMsg(d_msg);
}

bool CommunicationModule::srcPause() {
    LOG(INFO) << "CommunicationModule::srcPause()";
    return sendSimpleCommandMsg(sros::core::CMD_SRC_PAUSE);
}

bool CommunicationModule::srcContinue() {
    LOG(INFO) << "CommunicationModule::srcContinue()";
    return sendSimpleCommandMsg(sros::core::CMD_SRC_CONTINUE);
}

bool CommunicationModule::srcCancel() {
    LOG(INFO) << "CommunicationModule::srcCancel()";
    return sendSimpleCommandMsg(sros::core::CMD_COMMON_CANCEL);
}

bool CommunicationModule::startManualControl() {
    LOG(INFO) << "CommunicationModule::startManualControl()";
    return sendSimpleCommandMsg(sros::core::CMD_START_MANUAL_CONTROL);
}

bool CommunicationModule::stopManualControl() {
    LOG(INFO) << "CommunicationModule::stopManualControl()";
    return sendSimpleCommandMsg(sros::core::CMD_STOP_MANUAL_CONTROL);
}

bool CommunicationModule::setSpeedLevel(int level) {
    LOG(INFO) << "CommunicationModule::setSpeedLevel(): " << level;
    if (level < 1 || level > 100) {
        return false;
    }

    return sendSimpleCommandMsg(sros::core::CMD_SRC_SPEED_LEVEL, level);
}

bool CommunicationModule::setCurStation(uint16_t station_id) {
    LOG(INFO) << "CommunicationModule::setCurStation(): " << station_id;

    return sendSimpleCommandMsg(sros::core::CMD_SET_CUR_STATION, station_id);
}

bool CommunicationModule::triggerEmergency() {
    LOG(INFO) << "CommunicationModule::triggerEmergency()";
    return sendSimpleCommandMsg(sros::core::CMD_TRIGGER_EMERGENCY);
}

bool CommunicationModule::cancelEmergency() {
    LOG(INFO) << "CommunicationModule::cancelEmergency()";
    return sendSimpleCommandMsg(sros::core::CMD_CANCEL_EMERGENCY);
}

bool CommunicationModule::startCharge() {
    LOG(INFO) << "CommunicationModule::startCharge()";
    return sendSimpleCommandMsg(sros::core::CMD_ENABLE_AUTO_CHARGE);
}

bool CommunicationModule::stopCharge() {
    LOG(INFO) << "CommunicationModule::stopCharge()";
    return sendSimpleCommandMsg(sros::core::CMD_STOP_CHARGE);
}

bool CommunicationModule::enablePowerMode() {
    LOG(INFO) << "CommunicationModule::enablePowerMode()";
    return sendSimpleCommandMsg(sros::core::CMD_ENTER_POWER_SAVE_MODE);
}

bool CommunicationModule::disablePowerMode() {
    LOG(INFO) << "CommunicationModule::disablePowerMode()";
    return sendSimpleCommandMsg(sros::core::CMD_EXIT_POWER_SAVE_MODE);
}

bool CommunicationModule::restartSystem() {
    LOG(INFO) << "CommunicationModule::restartSystem()";
    return sendSimpleCommandMsg(sros::core::CMD_RESET_SROS);
}

bool CommunicationModule::setSRCSpeed(int16_t v_x, int16_t v_y, int16_t v_yaw) {
    return sendSimpleCommandMsg(sros::core::CMD_SET_SRC_SPEED, v_x, v_yaw);
}

bool CommunicationModule::goForward() {
    const int v_x = 100;  // 用chip手动控制，将速度调节到33%的速度

    return sendSimpleCommandMsg(sros::core::CMD_SET_SRC_SPEED, v_x, 0);
}

bool CommunicationModule::goBack() {
    const int v_x = -100;  // 用chip手动控制，将速度调节到33%的速度

    return sendSimpleCommandMsg(sros::core::CMD_SET_SRC_SPEED, v_x, 0);
}

bool CommunicationModule::goLeft() {
    const int w = 120;  // 用chip手动控制，将速度调节到33%的速度

    return sendSimpleCommandMsg(sros::core::CMD_SET_SRC_SPEED, 0, w);
}

bool CommunicationModule::goRight() {
    const int w = -120;  // 用chip手动控制，将速度调节到33%的速度

    return sendSimpleCommandMsg(sros::core::CMD_SET_SRC_SPEED, 0, w);
}

bool CommunicationModule::inputActionValue(int value) {
    return sendSimpleCommandMsg(sros::core::CMD_INPUT_ACTION_VALUE, 0, 0);
}

bool CommunicationModule::triggerFleetOnline() {
    //return sros::core::Settings::getInstance().setValue("main.fleet_mode", "FLEET_MODE_ONLINE");
    auto &s = sros::core::Settings::getInstance();
    auto item = s.getItemInfo("main.fleet_mode");
    item.value = "FLEET_MODE_ONLINE";
    return s.setItemInfo(item);
}

bool CommunicationModule::triggerFleetOffline() {
    //return sros::core::Settings::getInstance().setValue("main.fleet_mode", "FLEET_MODE_OFFLINE");
    auto &s = sros::core::Settings::getInstance();
    auto item = s.getItemInfo("main.fleet_mode");
    item.value = "FLEET_MODE_OFFLINE";
    return s.setItemInfo(item);
}

bool CommunicationModule::setOutGPIOBit(uint8_t index, bool enable) {
    uint8_t new_gpio_output_value = 0;
    if (enable) {
        new_gpio_output_value = g_state.gpio_output | (1 << index);
    } else {
        new_gpio_output_value = g_state.gpio_output & ~(1 << index);
    }
    return sendSimpleCommandMsg(sros::core::CMD_SET_GPIO_OUTPUT, new_gpio_output_value);
}

bool CommunicationModule::setOutGPIO(uint16_t value, uint16_t mask) {
    return sendSimpleCommandMsg(sros::core::CMD_SET_GPIO_OUTPUT, value, mask);
}

bool CommunicationModule::setVolume(uint16_t value) {
    LOG(INFO) << "CommunicationModule::setVolume()" << value;
    if (value > 100) {
        return false;
    }

    return sendSimpleCommandMsg(sros::core::CMD_SET_SPEAKER_VOLUME, value);
}

void CommunicationModule::onCommandResponseMsg(sros::core::base_msg_ptr m) {
    auto cmd_response = dynamic_pointer_cast<sros::core::CommandMsg>(m);
    if (cmd_response->source != getName()) {  // 这条命令不是由改模块发出的，所以拒收
        return;
    }

    result_state_ = cmd_response->result_state;
    result_code_ = cmd_response->result_code;
    response_received = true;

    LOG(INFO) << "result_state_: " << result_state_;
    LOG(INFO) << "result_code_: " << result_code_;
}

bool CommunicationModule::sendCommandMsg(core::base_msg_ptr m) {
    response_received = false;
    sendMsg(m);

    int counter = 0;
    while (!response_received) {
        if (!pocessOnceEvent()) {  // 在没有回到dispatch的情况下处理一次事件循环
            break;
        }

        if (++counter == 5) {  // 这种情况应该不存在
            LOG(ERROR) << "CommunicationModule 出现不回数据的情况 topic: " << m->topic_;
            return false;
        }
    }

    if (result_state_ == core::RESPONSE_OK || result_state_ == core::RESPONSE_PROCESSING) {
        return true;
    }

    return false;
}

void CommunicationModule::onNotifyMsg(sros::core::base_msg_ptr m) {
    auto notify_msg = dynamic_pointer_cast<sros::core::NotificationMsg>(m);

    if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED) {
        if (notify_msg->movement_task->getTaskSourceModule() == getName()) {
            onMoveTaskFinishedNotify(notify_msg);
        }
    } else if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED) {
        if (notify_msg->action_task->getTaskSourceModule() == getName()) {
            onActionTaskFinishedNotify(notify_msg);
        }
    }
}

void CommunicationModule::onMoveTaskFinishedNotify(core::NotificationMsg_ptr msg) {}

void CommunicationModule::onActionTaskFinishedNotify(core::NotificationMsg_ptr msg) {}

}  // namespace sros
