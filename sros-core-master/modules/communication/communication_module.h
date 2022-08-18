/**
 * @file communication_module.h
 *
 * @author pengjiali
 * @date 18-10-25.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MODULES_COMMUNICATION_COMMUNICATION_MODULE_H_
#define MODULES_COMMUNICATION_COMMUNICATION_MODULE_H_

#include "core/core.h"
#include "core/msg/command_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/pose.h"
#include "core/state.h"
#include "core/task/action_task.h"

namespace sros {

class CommunicationModule : public sros::core::Module {
 public:
    explicit CommunicationModule(const std::string &name);
    virtual ~CommunicationModule();

    void run() final;
    virtual bool subClassRunPrepare() = 0;  // 子类要绑定的topic等操作

 protected:
    bool moveToStation(core::TaskNo_t no, uint16_t station_id, core::ObstacleAvoidPolicy avoid_policy);
    bool moveToPose(core::TaskNo_t no, const core::Pose &pose, core::ObstacleAvoidPolicy avoid_policy);

    bool startNewAction(core::TaskNo_t no, int id, int param0, int param1);
    // 存在当前正在跑某个命令，然后另一个命令也能启动的情况
    // 例如（启动链板线，另外一个动作停止链板线，此时启动任务的时候不要判断是否有任务在执行）
    bool startNewCoexistAction(core::TaskNo_t no, int id, int param0, int param1);
    bool srcPause();
    bool srcContinue();
    bool srcCancel();

    bool startManualControl();
    bool stopManualControl();

    bool setSpeedLevel(int level);
    bool setCurStation(uint16_t station_id);
    bool setCurMap(const std::string &map_name);

    // 执行任务mission
    bool startMission(uint16_t mission_id);
    bool pauseMission();
    bool continueMission();
    bool cancelMission(uint64_t mission_uid);

    bool triggerEmergency();
    bool cancelEmergency();

    bool startCharge();
    bool stopCharge();

    bool startLocationByPose(const core::Pose &pose, bool absolute_location = false);
    bool startLocationByStation(uint16_t station_id);
    bool stopLocation();

    bool enablePowerMode();
    bool disablePowerMode();

    bool restartSystem();

    bool setSRCSpeed(int16_t v_x, int16_t v_y, int16_t v_yaw);
    bool goForward();
    bool goBack();
    bool goLeft();
    bool goRight();

    bool inputActionValue(int value = 0);

    bool triggerFleetOnline();
    bool triggerFleetOffline();

    bool setOutGPIOBit(uint8_t index, bool enable);
    bool setOutGPIO(uint16_t value, uint16_t mask = 0xFFFF);

    bool setVolume(uint16_t value);

    virtual void onMoveTaskFinishedNotify(core::NotificationMsg_ptr msg);
    virtual void onActionTaskFinishedNotify(core::NotificationMsg_ptr msg);

 private:
    inline bool sendSimpleCommandMsg(const core::CommandType &command_type, int param0 = 0, int param1 = 0,
                                     int param2 = 0);
    inline bool sendCommandMsg(core::base_msg_ptr m);

    void onCommandResponseMsg(sros::core::base_msg_ptr m);
    void onNotifyMsg(sros::core::base_msg_ptr m);

    bool response_received = false;  // 标记回复是否被接受
    core::ResultState result_state_ = core::RESPONSE_NONE;
    uint32_t result_code_ = core::ERROR_CODE_NONE;
};

}  // namespace sros

#endif  // MODULES_COMMUNICATION_COMMUNICATION_MODULE_H_
