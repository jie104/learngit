/**
 * @file vsc_module.h
 *
 * @author lhx
 * @date 17-02-17.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef MODULES_VSC_VSC_MODULE_H_
#define MODULES_VSC_VSC_MODULE_H_

#include <mutex>

#include "core/core.h"

#include "core/msg/str_msg.hpp"

#include "core/hardware/battery.h"
#include "core/hardware/PMU.h"
#include "core/msg/command_msg.hpp"
#include "core/task/task_manager.h"
#include "core/usart/connection.hpp"
#include "core/usart/frame_v1.h"
#include "modules/security/security_state_handle.h"
#include "vsc_upgrade.h"

namespace vsc {

using namespace std;

class VSCModule : public sros::core::Module {
 public:
    VSCModule();

    virtual ~VSCModule();

    virtual void run();

 private:
    bool enable_action_controller_;
    bool enable_safety_unit_;
    bool enable_power_unit_;
    bool enable_sonar_device_;  // 是否启用超声传感器
    bool enable_hmi_unit_;

    bool send_request_safety_;  // 是否在等待发送
    bool send_request_power_;   // 是否在等待发送

    enum SafetyUnitAction {
        SUA_NONE,
        SUA_RECOVER,
        SUA_TRIGGER,
    };

    SafetyUnitAction safety_action_;  // 安全单元需要发送的动作
    bool request_cancel_charge_action_;

    enum PowerUnitAction {
        PUA_NONE,
        PUA_START_CHARGE,
        PUA_STOP_CHARGE,
        PUA_ENTER_POWER_SAVE_MODE,
        PUA_EXIT_POWER_SAVE_MODE,
    };

    PowerUnitAction power_action_;  // 电源单元需要发送的动作

    uint64_t timer_cnt_;  // 100ms定时器计数

    uint64_t last_timer_send_data;  //记录上次发送数据的时间

    shared_ptr<usart::Connection<usart::FrameV1<>>> connection_ptr_;  // 串口通信抽象

    sros::core::BreakSwitchState cur_break_sw_state_;  // 解抱闸开关状态

    std::mutex lock_mutex_;

    sros::device::Device_ptr vsc_device_;
    std::shared_ptr<sros::device::Battery> battery_device_;

    int charge_detect_timeout_ = 15000;  // 检测是否充电成功的等待超时，单位ms

    bool is_vsc_hmi_disabled_ = false;

    bool is_send_sros_emergency_src_ = false;

    bool is_recv_vsc_version_ = false;

    int minute_of_mean_current_ = 5;  // 用最近几分钟的平均电流，来估算剩余电池电量使用时间

    // 电池最近电流列表，保持minute_of_mean_current_分钟，单位mA
    // 此处假定每个元素采样时间间隔为1s
    std::deque<int16_t> recent_discharge_battery_current_;

    void initUsartDevice();

    void scheduleTask();

    void onDebugCmdMsg(sros::core::base_msg_ptr msg);

    void onActionCmdMsg(sros::core::base_msg_ptr msg);

    void onVSCCommandMsg(sros::core::base_msg_ptr msg);

    void onTimer_50ms(sros::core::base_msg_ptr msg);

    void onTimer_20s(sros::core::base_msg_ptr msg);

    void onRecvSRCUsartData(std::vector<uint8_t> frame);

    void onRecvUsartData(const vector<uint8_t> &data);

    void sendUsartData(const vector<uint8_t> &data);

    void onSendUsartDataMsg(sros::core::base_msg_ptr msg);

    void handleSafetyUnitData(const vector<uint8_t> &data);

    void handlePowerUnitData(const vector<uint8_t> &data);

    void handleActionControllerData(const vector<uint8_t> &data);

    void handleSonarUnitData(const vector<uint8_t> &data);

    void handleSensorData(const vector<uint8_t> &data);

    void handleVSCData(const vector<uint8_t> &data);

    void updateHMIState();

    void handleChargeActionTask(sros::core::ActionTask_ptr action_task);

    //    void waitForFinishChargeCommand();

    void waitForFinishChargeActionTaskUntilPercentage();

    void waitForFinishBatteryMaintenanceActionTask();

    //    void waitForFinishCharge();

    void responseCommand(const sros::core::CommandMsg_ptr &msg);

    void updateNavigationState(sros::core::NavigationState state);

    void estimationBatteryRemainTime();  // 估算电池剩余使用时间

    void tempRemoveForkliftErr();

    void onUpdateVscMsg(sros::core::base_msg_ptr m);
    
    void onVSCUpgradeResult(int result);

    security::SecurityStateHandle security_state_handle_;

    VSCUpgrade vsc_upgrade_;
};

} /* namespace vsc */

#endif  // MODULES_VSC_VSC_MODULE_H_
