/*
* Author: Peng Lei
*
* Created on Mar 19, 2018, 11:30 AM
*/

#ifndef SROS_TSCREEN_MODULE_H
#define SROS_TSCREEN_MODULE_H

#include <stdint.h>
#include <string>
#include <vector>
#include "core/state.h"
#include "core/core.h"
#include "core/module.h"
#include "core/device/device.h"
#include "core/usart/timeout_serial.h"

namespace tscreen {

using namespace std;

enum SYSState {
    SYS_OK = 0,
    SYS_WAIT_FINISH_SLOW = 1,
    SYS_NAV_PAUSED = 2,
    SYS_HARDWARE_ERROR = 3,
    SYS_ENMERGENCY_MODE1 = 4,
    SYS_ENMERGENCY_MODE2 = 5,
};

enum TaskState {
    TS_IDLE = 0,
    TS_RUNNING = 1,
    TS_LOAD_FULL = 4,
    TS_LOAD_FREE = 5,
    TS_NO_PATH = 6,
    TS_LOAD_ERR = 7,
    TS_CHARGE_ERR = 8,
    TS_NO_DST_STATION = 9,
};

enum SignalStrength {
    SS_NONE = 0,
    SS_GOOD = 1,
    SS_MIDDLE = 2,
    SS_POOR = 3,
    SS_DISCONNECT = 4,
};

class ScreenModule : public sros::core::Module {
public:
    ScreenModule();

    virtual ~ScreenModule();

    virtual void run();

    void refreshUI();
    
private:
    std::vector<std::string> splitWithStl(const std::string &str,const std::string &pattern);
    uint16_t getCRCChecksum(const char *data, int len);
    bool checkRecvFrame(const vector<uint8_t> &data, const std::string &recv);
 
    uint16_t requestButtonState(uint16_t address, uint16_t value, uint8_t mode);
    bool updateScreenTEXT(uint16_t address, uint16_t value, uint8_t mode);
    bool update32ScreenTEXT(uint16_t address, uint32_t value);

    std::string VectorcharToString(const vector<uint8_t> &data);
    void printData(const vector<uint8_t> &data);
    void printDataStr(const std::string &data);

    void onTimer_1s(sros::core::base_msg_ptr m);
    void onTimer_50ms(sros::core::base_msg_ptr m);

    void makeActionTask(int action_p1, int action_p2, int action_p3);
    void makeMoveTask(int dst_station);

    SYSState pre_sys_state_; // 前一个系统状态
    SYSState sys_state_;
    TaskState pre_task_state_;
    TaskState task_state_;
    sros::device::Device_ptr touch_screen_device_;
    TimeoutSerial_ptr timeout_serial_;

    int pre_cur_v_;
    int last_update_dst_station_;
    int pre_total_mileage_;
    int pre_battery_percentage_;
    uint32_t pre_battery_remain_time_ = 0; // 电池预计剩余工作时间，单位min
    uint32_t pre_laster_error_code_ = 0;
    uint32_t pre_hardware_error_code_ = 0;
    int pre_operation_mode_;
    int pre_volume_value_;
    sros::core::LocationState pre_location_state_;
    sros::core::BatteryState pre_battery_state_;
    sros::core::PowerState pre_power_state_;
    uint64_t last_button_press_;
    int pre_station_;
    std::string local_ip_address_;
    std::string pre_local_ip_address_;
    std::string gateway_;

    bool is_action_task_enable_;
    bool is_move_task_enable_;
    int action_p1_;
    int action_p2_;
    int action_p3_;
    int move_dst_station_;
    int move_stay_time_;

    uint16_t realtime_alarm_code_; // 警告代码
    uint16_t pre_alarm_code_;

    bool is_speaker_on_;

    int cur_map_; // 当前地图index
    int pre_map_;

    std::string interface_;

    double pre_ping_ms_;
    double ping_ms_;
    bool is_device_online_;
    bool is_mix_task_running_;
    
    SignalStrength signal_;

    int low_battery_threshold_;

    bool is_tiean_manual_control_mode_ = false; // 标记是否是铁安手动控制模式

    std::map<uint16_t, bool> press_record_;
    void updatePressRecord(uint16_t id, uint16_t result);
    bool getPressRecord(uint16_t id);

    void updateButtonState(uint16_t value, uint16_t mode);
    void updateButtonsState();
    bool updateStateTEXT();
    void updateIPaddressTEXT();
    void autoStateShift();
    void updateState(sros::core::NavigationState state);
    void getPingMS();
    void updatePingValue(double value);
    void updatePingTimeout();
    void handleMixTask();
    void cancelActionTask();
    void initTEXT();
    void setNetworkInterfaceState(int state);
};

}
#endif
