/**
 * @file spu_module.h
 * @author cwt
 * @date 21-10-10.
 * @describe
 *      this is base on SPU100
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef  MODULES_SPU_SPU_MODULE_H_
#define MODULES_SPU_SPU_MODULE_H_

#include "core/core.h"
#include "core/msg/str_msg.hpp"
#include "core/hardware/battery.h"
#include "core/hardware/PMU.h"
#include "core/msg/command_msg.hpp"
#include "core/task/task_manager.h"
#include "core/usart/connection.hpp"
#include "core/usart/frame_v1.h"

namespace spu {

using namespace std;

class SPUModule : public sros::core::Module {
 public:
    SPUModule();
    virtual ~SPUModule();
    virtual void run();

 private:
    bool enable_power_unit_;
    bool send_request_power_;   // 是否在等待发送
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
    shared_ptr<usart::Connection<usart::FrameV1<>>> connection_ptr_;  // 串口通信抽象

    sros::device::Device_ptr spu_device_;
    std::shared_ptr<sros::device::Battery> battery_device_;
	bool enable_hmi_unit_;
	bool is_spu_hmi_disabled_ = false;
    int charge_detect_timeout_ = 15000;  // 检测是否充电成功的等待超时，单位ms
    bool is_recv_spu_version_ = false;
    int minute_of_mean_current_ = 5;  // 用最近几分钟的平均电流，来估算剩余电池电量使用时间

    // 电池最近电流列表，保持minute_of_mean_current_分钟，单位mA
    // 此处假定每个元素采样时间间隔为1s
    std::deque<int16_t> recent_discharge_battery_current_;

    void initUsartDevice();
    void onDebugCmdMsg(sros::core::base_msg_ptr msg);
    void onActionCmdMsg(sros::core::base_msg_ptr msg);
    void onTimer_50ms(sros::core::base_msg_ptr msg);
    void onTimer_20s(sros::core::base_msg_ptr msg);
    void onRecvSRCUsartData(std::vector<uint8_t> frame);
    void onRecvUsartData(const vector<uint8_t> &data);
    void sendUsartData(const vector<uint8_t> &data);
    void onSendUsartDataMsg(sros::core::base_msg_ptr msg);
    void sendBatteryRemainCapacityThreshold(int8_t  cmd, int32_t value);
    void sendHmiLcDeviceCanId(int8_t  cmd, int16_t value);

    void handlePowerUnitData(const vector<uint8_t> &data);
    void handleChargeActionTask(sros::core::ActionTask_ptr action_task);
    void handleSPUData(const vector<uint8_t> &data);
	  void handleSensorData(const vector<uint8_t> &data);
	  void updateHMIState();
    void onSPUCommandMsg(sros::core::base_msg_ptr msg);

    void waitForFinishChargeActionTaskUntilPercentage();
    void waitForFinishBatteryMaintenanceActionTask();

    void responseCommand(const sros::core::CommandMsg_ptr &msg);
    void estimationBatteryRemainTime();  // 估算电池剩余使用时间
    void tempRemoveForkliftErr();
};

} /* namespace spu*/

#endif  // MODULES_SPU_SPU_MODULE_H_
