//
// Created by penglei on 11/15/17.
//

#ifndef SROS_HMI_MODULE_H
#define SROS_HMI_MODULE_H

#include <stdint.h>
#include <map>
#include <string>
#include <vector>

#include "core/core.h"
#include "core/device/device.h"
#include "core/hardware/LC100.h"
#include "core/hardware/LC200.h"
#include "core/hardware/speaker.h"
#include "core/module.h"
#include "core/msg/command_msg.hpp"
#include "core/music_id.h"
#include "core/usart/socket_can.hpp"

namespace hmi {

using namespace std;

// 状态按照优先级越来越高排列
// 链接状态
enum class ConnectState {
    DISCONNECTED = 0x01,
    CONNECTED = 0x02,
};

// 电池状态
enum class BatteryState {
    NONE = 0x00,

    LOW_BATTERY = 0x01
};

// 普通的系统状态
enum class NormalSystemState {
    NONE = 0x00,

    MAP_SWITCHING = 0x01,  // 地图切换
};

// 用户设置的状态
enum class UserSetState {
    NONE = 0,

    WAITING_FOR_ELEVATOR = 1,  // 等待电梯
    DISINGECTING = 2,          // 正在消毒
    FMS_DISCONNECTED = 3,      // 调度系统掉线
    WHERE_AM_I = 4,            // 寻找机器人
};

// 定制化状态
enum class CustomState {
    NONE = 0x00,
    WAIT_FOR_MATCH_VSC_INFO = 0x01,    //等待匹配vsc扫码信息 
};

// 执行结果
enum class ResultState {
    NONE = 0x00,  // 没有结果

    SUCCEED = 0x01,  // 成功
    FAILED = 0x80,   // 大于0x80的都为失败
};

// 动作状态
enum class ActionState {
    NONE = 0,

    FORWARD = 0x10,
    BACKWARD = 0x11,
    LEFTWARD = 0x12,
    RIGHTWARD = 0x13,
    RUNNING,
};

// 运行状态
enum class MotionState {
    NONE = 0x00,  // 没有运行

    STILL = 0x01,
    FORWARD = 0x10,
    BACKWARD = 0x11,
    LEFTWARD = 0x12,
    RIGHTWARD = 0x13,
    SLOWDOWN_DIRECTION_UNKNOWN,
    SLOWDOWN_DIRECTION_FORWARD,
    SLOWDOWN_DIRECTION_BACKWARD,
    SLOWDOWN_DIRECTION_LEFT,
    SLOWDOWN_DIRECTION_RIGHT,
    SLOWDOWN_DIRECTION_LEFT_FORWARD,
    SLOWDOWN_DIRECTION_RIGHT_FORWARD,
    SLOWDOWN_DIRECTION_LEFT_BACKWARD,
    SLOWDOWN_DIRECTION_RIGHT_BACKWARD,
    // <- 消毒的优先级放到此处
    BLOCKED_DIRECTION_UNKNOWN,
    BLOCKED_DIRECTION_FORWARD,
    BLOCKED_DIRECTION_BACKWARD,
    BLOCKED_DIRECTION_LEFT,
    BLOCKED_DIRECTION_RIGHT,
    BLOCKED_DIRECTION_LEFT_FORWARD,
    BLOCKED_DIRECTION_RIGHT_FORWARD,
    BLOCKED_DIRECTION_LEFT_BACKWARD,
    BLOCKED_DIRECTION_RIGHT_BACKWARD,
    TRAFFIC_CONTROL,  // 交通管制
};

// 重要的系统状态
enum class SeriousSystemState {
    NONE = 0x00,
    PAUSE,
    FMS_DISCONNECT,
    LOCATE_ERROR,    // 定位出错需要特殊处理
    MANUAL_CONTROL,  // 手动控制
    BREAk_SW_NO,     // 解抱闸
    FAULT,           // 故障
    CHARGING,
    INITIALIZING,     // 初始化中
    BATTERY_TOO_LOW,  // 电池电量极低，即将强制关机
    POWER_SAVE_MODE,
    WARNING_TEMPERATURE,    //电池温度预警
    EMERGENCY_TEMPERATURE   //电池温度警告
};

enum TheMiddleLedType {
    TRICOLOR = 1,           // 三色灯
    EXTENSION = 2,          // 扩展灯（环形灯）
    TRICOLOR_TWO_WIRE = 3,  // 三色灯-两线
    AMR600,                 // AMR600的中间灯
    BUSINESS,               // 商用机器人
};

enum TaskResultSate {
    NONE = 0,
    AGV_TASK_ERROR = 1,
};

class HMIModule : public sros::core::Module {
 public:
    HMIModule();

    virtual ~HMIModule();

    virtual void run();

 private:
    bool enable_hmi_;

    void onTimer_1s(sros::core::base_msg_ptr m);

    void onTimer_200ms(sros::core::base_msg_ptr m);

    void onMusicMsg(sros::core::base_msg_ptr m);

    void onDebugCmdMsg(sros::core::base_msg_ptr msg);

    void onNotifyConnectedClinetCountMsg(sros::core::base_msg_ptr msg);

    void onNotifyTaskResultMsg(sros::core::base_msg_ptr msg);

    void updateState();  // 更新所有状态
    void setHMIState();  // 设置HMI的状态

    void playMusicItem(int id);

    bool sendCanMsg(uint32_t id, const std::vector<uint8_t> &data);

    void responseCommand(const sros::core::CommandMsg_ptr &msg);

    sros::device::Speaker_ptr speaker_ptr_;  // 喇叭设备

    int low_battery_threshold_ = 20;             // 可以配置
    const int very_low_battery_threshold_ = 10;  // 电池电量极低，即将强制关机

    int battery_emergency_temperature_; //电池告警温度阈值

    int battery_warning_temperature_;  //电池预警温度阈值

    int speaker_music_vol_;
    int cur_play_music_id_;
    int next_play_music_id_;  // 下一个需要播放的音乐

    bool is_playing_notify_music_;
    bool is_speaker_mute_;
    uint64_t last_sent_time_;

    int user_state_duration_time_ = 0;   // 用户状态持续时间(ms)
    int exec_result_duration_time_ = 0;  // 执行结果持续时间(ms)
    int custom_state_duration_time_ = 0;   // 自定义状态持续时间(ms)
    int error_music_id_ = 0;             // 错误需要播放的音乐ID
    int fault_music_id_ = 0;             // 故障需要播放的音乐ID
    int action_music_id_ = 0;
    int movement_forward_music_id_ = 0;

    // 所以状态，主要是平行状态，越往下优先级越高
    ConnectState connect_state_ = ConnectState::DISCONNECTED;
    BatteryState battery_state_ = BatteryState::NONE;
    UserSetState user_set_state_ = UserSetState::NONE;
    CustomState custom_state = CustomState::NONE;
    ResultState result_state_ = ResultState::NONE;
    ActionState action_state_ = ActionState::NONE;
    MotionState motion_state_ = MotionState::NONE;
    NormalSystemState normal_system_state_ = NormalSystemState::NONE;
    TaskResultSate action_or_motion_task_result_ = TaskResultSate::NONE;

    SeriousSystemState system_state_ = SeriousSystemState::NONE;

    sros::device::LC100_ptr lc100_ptr_agv_;                    // agv上的lc100，用于控制灯光
    sros::device::LC200_ptr lc200_ptr_agv_;                    // agv上的lc200，用于控制灯光
    const sros::device::LedNo LED_LEFT = sros::device::LED_1;  // 左转向灯
    const sros::device::LedNo LED_MIDDLE =
        sros::device::LED_2;  // 中间的灯光，（有些车没有接，华为R12项目中有，一个圆形的灯）有些可能是三色灯
    const sros::device::LedNo LED_RIGHT = sros::device::LED_3;  // 右转向灯
    TheMiddleLedType the_middle_led_type_ = TRICOLOR;           // 标记中间的灯的类型
};

} /* namespace hmi */
#endif
