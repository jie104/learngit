/*
 * Author: Peng Lei
 *
 * Created on Mar 19, 2018, 11:30 AM
 */

#include "screen_module.h"

#include <ifaddrs.h>
#include <fstream>
#include <iomanip>

#include <boost/crc.hpp>

#include "core/device/device_manager.h"
#include "core/fault_center.h"
#include "core/log/run_logger.h"
#include "core/map_manager.h"
#include "core/modbus/register_admin.h"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/hmi_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/task/task_manager.h"
#include "core/util/utils.h"
#include "core/version.h"
#include "modules/network/icmp/ping.hpp"

using namespace sros::core;

namespace tscreen {

#define WRITE_WORD 0x06
#define WRITE_BYTE 0x05
#define READ_BYTE 0x01
#define READ_WORD 0x03

#define SYS_OPERATION_MODE_ADDR 0x00C8
#define SYS_STATE_ADDR 0x00C9
#define SYS_SPEEP_ADDR 0x00CA
#define SYS_MAP_ADDR 0x00CB
#define SYS_LOCATE_STATE_ADDR 0x00CC
#define SYS_START_STATION_ADDR 0x00CD
#define SYS_END_STATION_ADDR 0x00CE
#define SYS_TASK_STATE_ADDR 0x00CF
#define SYS_CUR_STATION_ADDR 0x00D0
#define SYS_TOTAL_MILEAGE1_ADDR 0x00DC
#define SYS_TOTAL_MILEAGE2_ADDR 0x00DD
#define SYS_CUR_MILEAGE1_ADDR 0x00DE
#define SYS_CUR_MILEAGE2_ADDR 0x00DF
#define SYS_NETWORK_DELAY 0x00E5
#define SYS_IP1_ADDR 0x010E
#define SYS_IP2_ADDR 0x010F
#define SYS_IP3_ADDR 0x0110
#define SYS_IP4_ADDR 0x0111
#define SYS_BATTERY_PERCENTAGE_ADDR 0x00F0
#define SYS_VOLUME_VALUE_ADDR 0x00FA
#define SYS_CHARGE_STATE_ADDR 0x00C9
#define SYS_WAIT_CONFIRM_GOODS_ADDR 0x00CD
#define SYS_TASK_RUNNING_STATE_ADDR 0x00CA
#define SYS_SPEAKER_STATE_ADDR 0x00CB
#define SYS_REALTIME_ALARM_ADDR 0x0122
#define SYS_ACTION_UNIT_ADDR 0x00E4
#define SYS_NETWORK_STATE 0x00D2
#define SYS_POWER_MODE_ADDR 0x00CE
#define SYS_BATTERY_REMAIN_TIME 0x0190     // 电池电量剩余使用时间（分钟）
#define TIEAN_HEIGHT_INFO 0x0192           // 铁安顶升机构高度
#define TIEAN_OPERATION_MODE_ADDR 0x0194   // 铁安机构的手动控制状态[0x00: 自动模式, 0xFF: 手动模式]
#define SYS_LASTER_ERROR_CODE_ADDR 0x019A  // 错误码
#define SYS_HARDWARE_ERROR_CODE 0x019C     // 故障码

#define SYS_AGV_TYPE1_ADDR 0x012D
#define SYS_AGV_TYPE2_ADDR 0x012E
#define SYS_GROUP_NO_ADDR 0x012F
#define SYS_SRC_VERSION_ADDR 0x0130
#define SYS_SROS_VERSION_ADDR 0x0131
#define SYS_SERIAL_NO1_ADDR 0x0132
#define SYS_SERIAL_NO2_ADDR 0x0133

#define BTN_OPERATION_MODE 0x0064
#define BTN_START_LOC 0x0065
#define BTN_CANCEL_LOC 0x0066
#define BTN_START_CHARGE1 0x0067  // cancel then charge
#define BTN_START_CHARGE2 0x0068  // charge directly
#define BTN_SPEAKER 0x006A
#define BTN_CONTINUE 0x0078
#define BTN_PAUSE 0x0079
#define BTN_CANCEL 0x007A
#define BTN_CLEAR 0x007B
#define BTN_VOLUME_PLUS 0x006B
#define BTN_VOLUME_MINU 0x006C
#define BTN_EXCUTE_TASK 0x007C
#define BTN_ACTION_TYPE 0x0104
#define BTN_ACTION_ENABLE 0x006F
#define BTN_ACTION_P1 0x0066
#define BTN_ACTION_P2 0x0067
#define BTN_ACTION_P3 0x0068
#define BTN_CONFIRM_GOODS 0x0069
#define BTN_MTASK_ENABLE 0x006E
#define BTN_MTASK_STATION 0x0064
#define BTN_MTASK_STAYTIME 0x0065
#define BTN_TASK_CLEAR 0x0071
#define BTN_ENTER_POWER_SAVE_MODE 0x007E
#define BTN_EXIT_POWER_SAVE_MODE 0x007F
#define BTN_GO_STRAIGHT 0x0096
#define BTN_GO_BACK 0x0097
#define BTN_GO_LEFT 0x0098
#define BTN_GO_RIGHT 0x0099
#define BTN_REBOOT_DEVICE 0x009C
#define BTN_SET_EMERGENCY 0x009E
#define BTN_CANCEL_EMERGENCY 0x009D
#define BTN_LOCATE_STATION 0x006E
#define BTN_NETWORK_DISABLE 0x00B4
#define BTN_NETWORK_ENABLE 0x00B5
#define BTN_TIEAN_OPERATION_MODE 0x0195  // 切换铁安机构的手动控制模式

using namespace sros::device;

ScreenModule::ScreenModule()
    : Module("ScreenModule"),
      pre_sys_state_(SYS_OK),
      sys_state_(SYS_OK),
      pre_task_state_(TS_IDLE),
      task_state_(TS_IDLE),
      pre_cur_v_(-1),
      last_update_dst_station_(0),
      pre_total_mileage_(0),
      pre_battery_percentage_(0),
      pre_operation_mode_(-1),
      pre_volume_value_(30),
      last_button_press_(0),
      pre_location_state_(sros::core::LOCATION_STATE_NONE),
      pre_battery_state_(sros::core::BATTERY_NA),
      pre_station_(0),
      is_action_task_enable_(false),
      is_move_task_enable_(false),
      realtime_alarm_code_(0),
      is_speaker_on_(true),
      cur_map_(0),
      pre_map_(0),
      signal_(SS_NONE),
      is_device_online_(false),
      is_mix_task_running_(false) {}

ScreenModule::~ScreenModule() {}

void ScreenModule::run() {
    LOG(INFO) << "ScreenModule module start running";

    auto &s = sros::core::Settings::getInstance();

    auto enable_touch_screen = (s.getValue<std::string>("hmi.enable_touch_screen", "True") == "True");

    if (!enable_touch_screen) {
        LOG(ERROR) << "Touch screen module disable!";
        return;
    }

    low_battery_threshold_ = s.getValue<int>("main.low_battery_threshold", 15);

    auto touch_screen_version = s.getValue<std::string>("hmi.touch_screen_version", "v1");
    if (touch_screen_version == "v1") {
        touch_screen_device_ = DeviceManager::getInstance()->registerDevice(
            DEVICE_SCREEN, DEVICE_ID_SCREEN, DEVICE_COMM_INTERFACE_TYPE_RS232_1, DEVICE_MOUNT_SROS);
        CHECK(touch_screen_device_);

        auto touch_screen_device_name = s.getValue<std::string>("hmi.touch_screen_device_name", "/dev/ttyTHS1");
        auto touch_screen_device_rate = s.getValue<int>("hmi.touch_screen_device_rate", 115200);
        timeout_serial_.reset(new TimeoutSerial(touch_screen_device_name, touch_screen_device_rate));

        timeout_serial_->setTimeout(boost::posix_time::time_duration(0, 0, 2, 0));
        if (timeout_serial_->isOpen()) {
            LOG(INFO) << "Touch Screen serial device " << touch_screen_device_name << " rate "
                      << touch_screen_device_rate << " open succeed!";
            touch_screen_device_->setStateOK();
        } else {
            LOG(ERROR) << "Touch Screen serial device " << touch_screen_device_name << " open failed!";
            touch_screen_device_->setStateOpenFailed();
            return;
        }
    } else if (touch_screen_version == "v2") {
        LOGGER(INFO, SROS) << "Use touch screen version v2";
        bool enable_modbus_tcp = (s.getValue<std::string>("modbus.enable_modbus_tcp", "False") == "True");
        if (!enable_modbus_tcp) {
            LOGGER(ERROR, SROS) << "Touch screen v2 must start modbus tcp, modbus.enable_modbus_tcp need be True!";
        } else {
            // screen_v2进程实例已经在startup中已经启动
            // FIXME（pengjiali）：进程通信
            // 由于和触摸屏直接通信用的是python，sros无法拿到触摸屏的链接状态，所有v2没有触摸屏设备。后期需要考虑进程通信，或者是modbus登录来实现监听触摸屏是否链接上
        }
        stop();
        return;
    }

    interface_ = s.getValue<std::string>("network.communication_interface", "eth0");

    subscribeTopic("TIMER_1S", CALLBACK(&ScreenModule::onTimer_1s));
    subscribeTopic("TIMER_50MS", CALLBACK(&ScreenModule::onTimer_50ms));

    initTEXT();
    // boost::thread(boost::bind(&ScreenModule::autoStateShift, this));
    boost::thread(boost::bind(&ScreenModule::getPingMS, this));

    dispatch();
}

void ScreenModule::initTEXT() {
    auto &s = sros::core::Settings::getInstance();

    if (g_state.location_state == LOCATION_STATE_NONE) {
        updateScreenTEXT(SYS_LOCATE_STATE_ADDR, 0, WRITE_WORD);
    }

    updateIPaddressTEXT();

    auto action_unit = s.getValue<std::string>("main.action_unit", "riser");
    if (action_unit == "riser") {
        updateScreenTEXT(SYS_ACTION_UNIT_ADDR, 1, WRITE_WORD);
    } else if (action_unit == "roller") {
        updateScreenTEXT(SYS_ACTION_UNIT_ADDR, 3, WRITE_WORD);
    } else if (action_unit == "droller") {
        updateScreenTEXT(SYS_ACTION_UNIT_ADDR, 4, WRITE_WORD);
    } else {
        updateScreenTEXT(SYS_ACTION_UNIT_ADDR, 5, WRITE_WORD);
    }

    // init button
    updateScreenTEXT(SYS_STATE_ADDR, SYS_OK, WRITE_WORD);
    updateScreenTEXT(SYS_OPERATION_MODE_ADDR, 0xff, WRITE_BYTE);
    updateScreenTEXT(SYS_MAP_ADDR, 0, WRITE_WORD);

    updateScreenTEXT(SYS_SROS_VERSION_ADDR, SROS_MAJOR_VERSION * 100 + SROS_MINOR_VERSION * 10 + SROS_PATCH_VERSION,
                     WRITE_WORD);

    auto src_version_no = src_sdk->getSRCVersion();
    int major = src_version_no / (1000 * 1000);
    int minor = (src_version_no / 1000) % 1000;
    int patch = src_version_no % 1000;
    updateScreenTEXT(SYS_SRC_VERSION_ADDR, major * 100 + minor * 10 + patch, WRITE_WORD);
}

void ScreenModule::onTimer_1s(sros::core::base_msg_ptr m) {
    sros::core::MapManager map_manager;
    auto map_list = map_manager.getMapListSortByName();
    int i = 0;
    for (auto map_name : map_list) {
        if (g_state.getCurMapName() == map_name) {
            cur_map_ = i + 1;
            break;
        }
        ++i;
    }

    local_ip_address_ = g_state.ip_addr.get();

    // update realtime_alarm_code_
    if (g_state.location_state == LOCATION_STATE_ERROR) {
        realtime_alarm_code_ |= 0x0002;
    } else {
        realtime_alarm_code_ &= ~0x0002;
    }

    if (g_state.sys_state == SYS_STATE_TASK_NAV_NO_WAY) {
        realtime_alarm_code_ |= 0x0004;
    } else {
        realtime_alarm_code_ &= ~0x0004;
    }

    // reserved 0x0008 0x0010

    if (g_state.sys_state == SYS_STATE_TASK_NAV_NO_STATION) {
        realtime_alarm_code_ |= 0x0020;
    } else {
        realtime_alarm_code_ &= ~0x0020;
    }

    if (g_state.sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW ||
        g_state.sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW) {
        realtime_alarm_code_ |= 0x0040;
    } else {
        realtime_alarm_code_ &= ~0x0040;
    }

    if (g_state.sys_state == SYS_STATE_TASK_NAV_PAUSED || g_state.sys_state == SYS_STATE_TASK_PATH_PAUSED) {
        realtime_alarm_code_ |= 0x0080;
    } else {
        realtime_alarm_code_ &= ~0x0080;
    }

    if (g_state.isLaserError()) {
        realtime_alarm_code_ |= 0x0100;
    } else {
        realtime_alarm_code_ &= ~0x0100;
    }

    if (g_state.isSrcError()) {
        realtime_alarm_code_ |= 0x0200;
    } else {
        realtime_alarm_code_ &= ~0x0200;
    }

    if (g_state.isMotor1Error() || g_state.isMotor2Error() || g_state.isMotor3Error() || g_state.isMotor4Error()) {
        realtime_alarm_code_ |= 0x0400;
    } else {
        realtime_alarm_code_ &= ~0x0400;
    }

    if (g_state.emergency_state == sros::core::STATE_EMERGENCY_TRIGER ||
        g_state.emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE) {

        if (g_state.emergency_source >= sros::core::EMERGENCY_SRC_EDGE_1 &&
            g_state.emergency_source <= sros::core::EMERGENCY_SRC_EDGE_4) {
            realtime_alarm_code_ |= 0x1000;
        } else if (g_state.emergency_source >= sros::core::EMERGENCY_SRC_BUTTON_1 &&
                   g_state.emergency_source <= sros::core::EMERGENCY_SRC_BUTTON_4) {
            realtime_alarm_code_ |= 0x0800;
        } else if (g_state.emergency_source == sros::core::EMERGENCY_SRC_SAFETY_LIDAR) {
            realtime_alarm_code_ |= 0x4000;
        } else {
            realtime_alarm_code_ |= 0x1000;
        }
    } else {
        if ((realtime_alarm_code_ & 0x0800) == 0x0800) {
            realtime_alarm_code_ &= ~0x0800;
        } else if ((realtime_alarm_code_ & 0x1000) == 0x1000) {
            realtime_alarm_code_ &= ~0x1000;
        } else if((realtime_alarm_code_ & 0x4000) == 0x4000) {
            realtime_alarm_code_ &= ~0x4000;
        }
    }

    if (g_state.battery_percentage <= low_battery_threshold_) {
        realtime_alarm_code_ |= 0x2000;
    } else {
        realtime_alarm_code_ &= ~0x2000;
    }

    // 更新铁安机构的高度
    const uint16_t ADDR_TIEAN_HEIGHT_INFO_STATUS = 0x4000;
    int tiean_height = 0;
    bool ret = src_sdk->getParameter(ADDR_TIEAN_HEIGHT_INFO_STATUS, tiean_height, 50, true);
    if (ret) {
        // FIXME(pengjiali): 给src改成modbus方式时，将此函数放到src中去一起处理
        RegisterAdmin::getInstance()->setInputRegister16(sros::IRA_TIE_AN_JACK_HEIGHT, tiean_height);
        updateScreenTEXT(TIEAN_HEIGHT_INFO, tiean_height, WRITE_WORD);
    }
}

void ScreenModule::onTimer_50ms(sros::core::base_msg_ptr m) { refreshUI(); }

void ScreenModule::refreshUI() {
    if (updateStateTEXT()) {
        updateButtonsState();

        updateIPaddressTEXT();

        static int i = 0;
        ++i;
        if (i % 10 == 0 && touch_screen_device_) {
            touch_screen_device_->keepAlive();
        }
    }
}

bool ScreenModule::updateStateTEXT() {
    // SYS_STATE
    pre_sys_state_ = sys_state_;
    if (g_state.sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW ||
        g_state.sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW) {
        sys_state_ = SYS_WAIT_FINISH_SLOW;
    } else if (g_state.sys_state == SYS_STATE_TASK_NAV_PAUSED || g_state.sys_state == SYS_STATE_TASK_PATH_PAUSED) {
        sys_state_ = SYS_NAV_PAUSED;
    } else if (g_state.sys_state == SYS_STATE_HARDWARE_ERROR) {
        sys_state_ = SYS_HARDWARE_ERROR;
    } else if (g_state.emergency_state != STATE_EMERGENCY_TRIGER &&
               g_state.emergency_state != STATE_EMERGENCY_RECOVERABLE) {
        sys_state_ = SYS_OK;
    }

    if (g_state.emergency_state == sros::core::STATE_EMERGENCY_TRIGER ||
        g_state.emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE) {
        if (g_state.emergency_source >= sros::core::EMERGENCY_SRC_EDGE_1 &&
            g_state.emergency_source <= sros::core::EMERGENCY_SRC_EDGE_4) {
            sys_state_ = SYS_ENMERGENCY_MODE2;
        } else if (g_state.emergency_source >= sros::core::EMERGENCY_SRC_BUTTON_1 &&
                   g_state.emergency_source >= sros::core::EMERGENCY_SRC_BUTTON_4) {
            sys_state_ = SYS_ENMERGENCY_MODE1;
        }
    } else if (g_state.sys_state != SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW &&
               g_state.sys_state != SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW &&
               g_state.sys_state != SYS_STATE_TASK_NAV_PAUSED && g_state.sys_state != SYS_STATE_TASK_PATH_PAUSED &&
               g_state.sys_state != SYS_STATE_HARDWARE_ERROR) {
        sys_state_ = SYS_OK;
    }
    // TASK_STATE
    pre_task_state_ = task_state_;
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (g_state.sys_state == SYS_STATE_TASK_NAV_NO_WAY) {
        task_state_ = TS_NO_PATH;
    } else if (g_state.sys_state == SYS_STATE_TASK_NAV_NO_STATION) {
        task_state_ = TS_NO_DST_STATION;
    } else if (g_state.sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH ||
               g_state.sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH ||
               g_state.sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW ||
               g_state.sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW ||
               g_state.sys_state == SYS_STATE_TASK_NAV_PAUSED || g_state.sys_state == SYS_STATE_TASK_PATH_PAUSED) {
        task_state_ = TS_RUNNING;
    } else if (!action_task || !action_task->isRunning()) {
        task_state_ = TS_IDLE;
    }

    if (action_task && action_task->isRunning()) {
        if (action_task->getLoadingState() == sros::core::LOADING_FREE) {
            task_state_ = TS_LOAD_FREE;
        } else if (action_task->getLoadingState() == sros::core::LOADING_FULL) {
            task_state_ = TS_LOAD_FULL;
        }
    } else if (g_state.sys_state != SYS_STATE_TASK_NAV_NO_WAY && g_state.sys_state != SYS_STATE_TASK_NAV_NO_STATION &&
               g_state.sys_state != SYS_STATE_TASK_PATH_WAITING_FINISH &&
               g_state.sys_state != SYS_STATE_TASK_NAV_WAITING_FINISH &&
               g_state.sys_state != SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW &&
               g_state.sys_state != SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW &&
               g_state.sys_state != SYS_STATE_TASK_NAV_PAUSED && g_state.sys_state != SYS_STATE_TASK_PATH_PAUSED) {
        task_state_ = TS_IDLE;
    }

    const int ACTION_ID_WAIT_COMMAND = 131;
    if (action_task && action_task->isRunning() && action_task->getActionID() == ACTION_ID_WAIT_COMMAND) {
        // 等待按下“确认货物”按钮
        if (!updateScreenTEXT(SYS_WAIT_CONFIRM_GOODS_ADDR, 0xff, WRITE_BYTE)) {
            return false;
        }
    } else {
        if (!updateScreenTEXT(SYS_WAIT_CONFIRM_GOODS_ADDR, 0x00, WRITE_BYTE)) {
            return false;
        }
    }

    if (action_task && action_task->isRunning() && action_task->getActionID() == 15 &&
        action_task->getActionParam() == 2 && action_task->getActionParam1() == 0) {  // 正在执行铁安机构手动控制
        is_tiean_manual_control_mode_ = true;
        if (!updateScreenTEXT(TIEAN_OPERATION_MODE_ADDR, 0xff, WRITE_BYTE)) {
            return false;
        }
    } else {
        if (!updateScreenTEXT(TIEAN_OPERATION_MODE_ADDR, 0x00, WRITE_BYTE)) {
            is_tiean_manual_control_mode_ = false;
            return false;
        }
    }

    auto move_task = sros::core::TaskManager::getInstance()->getMovementTask();
    if (move_task && move_task->isRunning() && last_update_dst_station_ != move_task->getCurDstStation()) {
        updateScreenTEXT(SYS_START_STATION_ADDR, move_task->getCurStartStation(), WRITE_WORD);
        updateScreenTEXT(SYS_END_STATION_ADDR, move_task->getCurDstStation(), WRITE_WORD);
        last_update_dst_station_ = move_task->getCurDstStation();
    } else if (last_update_dst_station_ == 0) {
        updateScreenTEXT(SYS_START_STATION_ADDR, 0, WRITE_WORD);
        updateScreenTEXT(SYS_END_STATION_ADDR, 0, WRITE_WORD);
        last_update_dst_station_ = 0;
    } else if (move_task && !move_task->isRunning() && last_update_dst_station_ != 0) {
        updateScreenTEXT(SYS_START_STATION_ADDR, 0, WRITE_WORD);
        updateScreenTEXT(SYS_END_STATION_ADDR, 0, WRITE_WORD);
        last_update_dst_station_ = 0;
    }

    if (pre_sys_state_ != sys_state_) {
        // LOG(INFO) << "write word sys_state_ = "<<sys_state_;
        updateScreenTEXT(SYS_STATE_ADDR, sys_state_, WRITE_WORD);
    }

    if (pre_task_state_ != task_state_) {
        // LOG(INFO) << "write word task_state_ = "<<task_state_;
        if (task_state_ == TS_LOAD_FULL || task_state_ == TS_LOAD_FREE || task_state_ == TS_RUNNING) {
            updateScreenTEXT(SYS_TASK_RUNNING_STATE_ADDR, 0xff, WRITE_BYTE);
        } else {
            updateScreenTEXT(SYS_TASK_RUNNING_STATE_ADDR, 0x00, WRITE_BYTE);
        }
        updateScreenTEXT(SYS_TASK_STATE_ADDR, task_state_, WRITE_WORD);
    }

    if (pre_cur_v_ != g_src_state.cur_v) {
        updateScreenTEXT(SYS_SPEEP_ADDR, g_src_state.cur_v, WRITE_WORD);
        pre_cur_v_ = g_src_state.cur_v;
    }

    if (pre_map_ != cur_map_) {
        updateScreenTEXT(SYS_MAP_ADDR, cur_map_, WRITE_WORD);
        pre_map_ = cur_map_;
    }

    auto &run_logger = sros::core::RunLogger::getInstance();
    auto total_mileages = run_logger.getTotalMileage();
    total_mileages = total_mileages / 1000;  // km
    if (pre_total_mileage_ != total_mileages) {
        updateScreenTEXT(SYS_TOTAL_MILEAGE1_ADDR, total_mileages, WRITE_WORD);
        pre_total_mileage_ = total_mileages;
    }

    if (pre_battery_percentage_ != g_state.battery_percentage) {
        updateScreenTEXT(SYS_BATTERY_PERCENTAGE_ADDR, g_state.battery_percentage, WRITE_WORD);
        pre_battery_percentage_ = g_state.battery_percentage;
    }

    if (pre_battery_remain_time_ != g_state.battery_remain_time) {
        updateScreenTEXT(SYS_BATTERY_REMAIN_TIME, g_state.battery_remain_time, WRITE_WORD);
        pre_battery_remain_time_ = g_state.battery_remain_time;
    }

    if (pre_laster_error_code_ != g_state.laster_error_code) {
        LOG(INFO) << "laster_error_code " << g_state.laster_error_code;
        update32ScreenTEXT(SYS_LASTER_ERROR_CODE_ADDR, g_state.laster_error_code);
        pre_laster_error_code_ = g_state.laster_error_code;
    }

    auto first_fault = FaultCenter::getInstance()->getFirstFault();
    if (first_fault == nullptr) {
        if (pre_hardware_error_code_ != 0) {
            update32ScreenTEXT(SYS_HARDWARE_ERROR_CODE, 0);
            pre_hardware_error_code_ = 0;
        }
    } else {
        auto first_fault_code = (uint32_t)first_fault->id;
        if (pre_hardware_error_code_ != first_fault_code) {
            update32ScreenTEXT(SYS_HARDWARE_ERROR_CODE, first_fault_code);
            pre_hardware_error_code_ = first_fault_code;
        }
    }

    if (g_state.operation_state == sros::core::OPERATION_MANUAL && pre_operation_mode_ != 0xff) {
        updateScreenTEXT(SYS_OPERATION_MODE_ADDR, 0xff, WRITE_BYTE);
        pre_operation_mode_ = 0xff;
    } else if (g_state.operation_state == sros::core::OPERATION_AUTO && pre_operation_mode_ != 0x00) {
        updateScreenTEXT(SYS_OPERATION_MODE_ADDR, 0x00, WRITE_BYTE);
        pre_operation_mode_ = 0x00;
    }

    if (pre_volume_value_ != g_state.cur_volume) {
        updateScreenTEXT(SYS_VOLUME_VALUE_ADDR, g_state.cur_volume, WRITE_WORD);
        if (g_state.cur_volume == 0) {
            is_speaker_on_ = false;
            updateScreenTEXT(SYS_SPEAKER_STATE_ADDR, 0x00, WRITE_BYTE);  // turn off
        } else {
            is_speaker_on_ = true;
            updateScreenTEXT(SYS_SPEAKER_STATE_ADDR, 0xff, WRITE_BYTE);  // turn on
        }
        pre_volume_value_ = g_state.cur_volume;
    }

    // location state
    if (g_state.location_state == LOCATION_STATE_NONE && pre_location_state_ != LOCATION_STATE_NONE) {
        updateScreenTEXT(SYS_LOCATE_STATE_ADDR, 0, WRITE_WORD);
        pre_location_state_ = g_state.location_state;
    } else if (g_state.location_state == LOCATION_STATE_RUNNING && pre_location_state_ != LOCATION_STATE_RUNNING) {
        updateScreenTEXT(SYS_LOCATE_STATE_ADDR, 1, WRITE_WORD);
        pre_location_state_ = g_state.location_state;
    } else if (g_state.location_state == LOCATION_STATE_INITIALING &&
               pre_location_state_ != LOCATION_STATE_INITIALING) {
        updateScreenTEXT(SYS_LOCATE_STATE_ADDR, 2, WRITE_WORD);
        pre_location_state_ = g_state.location_state;
    } else if (g_state.location_state == LOCATION_STATE_ERROR && pre_location_state_ != LOCATION_STATE_ERROR) {
        updateScreenTEXT(SYS_LOCATE_STATE_ADDR, 3, WRITE_WORD);
        pre_location_state_ = g_state.location_state;
    }

    if (g_state.battery_state == sros::core::BATTERY_CHARGING && pre_battery_state_ != sros::core::BATTERY_CHARGING) {
        updateScreenTEXT(SYS_CHARGE_STATE_ADDR, 0xff, WRITE_BYTE);
        pre_battery_state_ = g_state.battery_state;
    } else if ((g_state.battery_state == sros::core::BATTERY_NO_CHARGING ||
                g_state.battery_state == sros::core::BATTERY_NA) &&
               (pre_battery_state_ == sros::core::BATTERY_CHARGING)) {
        updateScreenTEXT(SYS_CHARGE_STATE_ADDR, 0x00, WRITE_BYTE);
        pre_battery_state_ = g_state.battery_state;
    }

    if (g_state.power_state == sros::core::POWER_SAVE_MODE && g_state.power_state != pre_power_state_) {
        updateScreenTEXT(SYS_POWER_MODE_ADDR, 0xff, WRITE_BYTE);
    } else if (g_state.power_state != pre_power_state_) {
        updateScreenTEXT(SYS_POWER_MODE_ADDR, 0x00, WRITE_BYTE);
    }
    pre_power_state_ = g_state.power_state;

    if (pre_station_ != g_state.station_no) {
        updateScreenTEXT(SYS_CUR_STATION_ADDR, g_state.station_no, WRITE_WORD);
        pre_station_ = g_state.station_no;
    }

    if (realtime_alarm_code_ > 1) {
        realtime_alarm_code_ |= 0x0001;
    } else {
        realtime_alarm_code_ &= ~0x0001;
    }

    if (pre_alarm_code_ != realtime_alarm_code_) {
        updateScreenTEXT(SYS_REALTIME_ALARM_ADDR, realtime_alarm_code_, WRITE_WORD);
        pre_alarm_code_ = realtime_alarm_code_;
    }

    if (is_device_online_) {
        updateScreenTEXT(SYS_NETWORK_STATE, 0xff, WRITE_BYTE);
    } else {
        updateScreenTEXT(SYS_NETWORK_STATE, 0x00, WRITE_BYTE);
    }

    if (pre_ping_ms_ != ping_ms_) {
        updateScreenTEXT(SYS_NETWORK_DELAY, ping_ms_, WRITE_WORD);
        pre_ping_ms_ = ping_ms_;
    }

    return true;
}

void ScreenModule::updateButtonState(uint16_t value, uint16_t mode) {
    // BTN_OPERATION_MODE
    uint16_t operation_result = requestButtonState(value, 1, mode);
    auto msg1 = make_shared<sros::core::CommandMsg>(getName());
    auto msg2 = make_shared<sros::core::HmiMsg>();
    auto msg3 = make_shared<sros::core::CommandMsg>(getName());
    bool is_msg1_shoud_be_sent = false;
    bool is_msg2_shoud_be_sent = false;
    bool is_msg3_shoud_be_sent = false;
    if (operation_result != 0) {
        switch (value) {
            case BTN_OPERATION_MODE: {
                if (getPressRecord(value)) {
                    return;
                }
                if (g_state.operation_state == sros::core::OPERATION_AUTO) {
                    LOG(INFO) << "MANUAL CONTROL pressed!";
                    updateScreenTEXT(SYS_OPERATION_MODE_ADDR, 0xff, WRITE_BYTE);
                    msg1->command = sros::core::CMD_START_MANUAL_CONTROL;
                    is_msg1_shoud_be_sent = true;
                } else if (g_state.operation_state == sros::core::OPERATION_MANUAL) {
                    LOG(INFO) << "AUTO CONTROL pressed!";
                    updateScreenTEXT(SYS_OPERATION_MODE_ADDR, 0x00, WRITE_BYTE);
                    msg1->command = sros::core::CMD_STOP_MANUAL_CONTROL;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_START_LOC: {
                if (!getPressRecord(value)) {
                    msg1->command = sros::core::CMD_START_LOCATION;
                    auto station = requestButtonState(BTN_LOCATE_STATION, 1, READ_WORD);
                    LOG(INFO) << "START LOCATE pressed! station = " << station;
                    msg1->param0 = station;
                    is_msg1_shoud_be_sent = true;

                    updateScreenTEXT(BTN_LOCATE_STATION, 0x00, WRITE_WORD);
                }
                break;
            }
            case BTN_CANCEL_LOC: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "CANCEL LOCATE pressed!";
                    msg1->command = sros::core::CMD_STOP_LOCATION;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_SPEAKER: {
                if (getPressRecord(value)) {
                    return;
                }

                if (is_speaker_on_) {
                    LOG(INFO) << "TURN OFF SPEAKER pressed!";
                    updateScreenTEXT(SYS_SPEAKER_STATE_ADDR, 0x00, WRITE_BYTE);  // turn off
                    msg2->command = HMI_COMMAND_MUTE;
                    is_msg2_shoud_be_sent = true;
                    is_speaker_on_ = false;
                } else {
                    LOG(WARNING) << "TURN ON SPEAKER pressed!";
                    updateScreenTEXT(SYS_SPEAKER_STATE_ADDR, 0xff, WRITE_BYTE);  // turn on
                    msg2->command = HMI_COMMAND_MUTE_CANCEL;
                    is_msg2_shoud_be_sent = true;
                    is_speaker_on_ = true;
                }
                break;
            }
            case BTN_CONTINUE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "CONTINUE pressed!";
                    msg1->command = sros::core::CMD_SRC_CONTINUE;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_PAUSE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "PAUSE pressed!";
                    msg1->command = sros::core::CMD_SRC_PAUSE;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_CANCEL: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "CANCEL pressed!";
                    msg1->command = CMD_COMMON_CANCEL;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_CLEAR: {
                if (!getPressRecord(value)) {
                    // TODO: 将解除急停单独提取出来
                    LOG(INFO) << "CLEAR pressed!";
                    //                    // cancel movetask
                    //                    msg3->network_task = sros::core::NETWORK_TASK_CANCEL;
                    //                    is_msg3_shoud_be_sent = true;
                    //                    // cancel action task
                    //                    cancelActionTask();

                    msg1->command = sros::core::CMD_CANCEL_EMERGENCY;
                    is_msg1_shoud_be_sent = true;

                    msg3->command = CMD_RESET_FAULT;
                    is_msg3_shoud_be_sent = true;
                }
                break;
            }
            case BTN_VOLUME_PLUS: {
                const int VOLUME_ADJUST_STEP = 10;
                // LOG(INFO) << "INCREASE VOLUME pressed!";
                if (g_state.cur_volume < 100) {
                    msg1->command = sros::core::CMD_SET_SPEAKER_VOLUME;
                    auto new_volume = g_state.cur_volume + VOLUME_ADJUST_STEP;
                    if (new_volume > 100) new_volume = 100;
                    msg1->param0 = new_volume;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_VOLUME_MINU: {
                const int VOLUME_ADJUST_STEP = 10;
                // LOG(INFO) << "DECREASE VOLUME pressed!";
                if (g_state.cur_volume > 0) {
                    msg1->command = sros::core::CMD_SET_SPEAKER_VOLUME;
                    if (g_state.cur_volume <= VOLUME_ADJUST_STEP) {
                        msg1->param0 = 0;
                    } else {
                        msg1->param0 = g_state.cur_volume - VOLUME_ADJUST_STEP;
                    }
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_CONFIRM_GOODS: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "BTN_CONFIRM_GOODS pressed!";
                    msg1->command = sros::core::CMD_INPUT_ACTION_VALUE;
                    msg1->param0 = 0;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_ENTER_POWER_SAVE_MODE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "BTN_ENTER_POWER_SAVE_MODE pressed!";
                    msg1->command = sros::core::CMD_ENTER_POWER_SAVE_MODE;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_EXIT_POWER_SAVE_MODE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "BTN_EXIT_POWER_SAVE_MODE pressed!";
                    msg1->command = sros::core::CMD_EXIT_POWER_SAVE_MODE;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_ACTION_ENABLE: {
                // if (getPressRecord(value)) return;
                is_action_task_enable_ = true;
                action_p1_ = requestButtonState(BTN_ACTION_P1, 1, READ_WORD);
                action_p2_ = requestButtonState(BTN_ACTION_P2, 1, READ_WORD);
                action_p3_ = requestButtonState(BTN_ACTION_P3, 1, READ_WORD);
                break;
            }
            case BTN_MTASK_ENABLE: {
                // if (getPressRecord(value)) return;
                is_move_task_enable_ = true;
                move_dst_station_ = requestButtonState(BTN_MTASK_STATION, 1, READ_WORD);
                move_stay_time_ = requestButtonState(BTN_MTASK_STAYTIME, 1, READ_WORD);
                break;
            }
            case BTN_EXCUTE_TASK: {
                if (getPressRecord(value)) {
                    break;
                }
                LOG(INFO) << "EXCUTE pressed!";
                if (is_move_task_enable_ || is_action_task_enable_) {
                    if (is_action_task_enable_ && !is_move_task_enable_) {
                        makeActionTask(action_p1_, action_p2_, action_p3_);
                    } else if (!is_action_task_enable_ && is_move_task_enable_ && move_dst_station_ > 0) {
                        makeMoveTask(move_dst_station_);
                    } else {
                        // LOG(ERROR) <<" unspported task combination!!!";
                        boost::thread(boost::bind(&ScreenModule::handleMixTask, this));
                    }
                }
                break;
            }
            case BTN_TASK_CLEAR: {
                if (getPressRecord(value)) {
                    break;
                }
                LOG(INFO) << "TASK CLEAR pressed ";
                updateScreenTEXT(BTN_ACTION_P1, 0, WRITE_WORD);
                updateScreenTEXT(BTN_ACTION_P2, 0, WRITE_WORD);
                updateScreenTEXT(BTN_ACTION_P3, 0, WRITE_WORD);

                updateScreenTEXT(BTN_MTASK_STATION, 0, WRITE_WORD);
                updateScreenTEXT(BTN_MTASK_STAYTIME, 0, WRITE_WORD);

                updateScreenTEXT(BTN_MTASK_ENABLE, 0, WRITE_BYTE);
                updateScreenTEXT(BTN_ACTION_ENABLE, 0, WRITE_BYTE);
                break;
            }
            case BTN_CANCEL_EMERGENCY: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "EMERGENCY pressed!";
                    msg1->command = sros::core::CMD_CANCEL_EMERGENCY;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_REBOOT_DEVICE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "REBOOT!!!!";
                    msg1->command = sros::core::CMD_RESET_SROS;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }

            case BTN_START_CHARGE1: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "cancel current task then go to charge station!";
                    msg1->command = CMD_ENABLE_AUTO_CHARGE;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }

            case BTN_START_CHARGE2: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "go to charge station!";
                    msg1->command = CMD_ENABLE_AUTO_CHARGE;
                    is_msg1_shoud_be_sent = true;
                }
                break;
            }
            case BTN_NETWORK_ENABLE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "Enable network interface";

                    setNetworkInterfaceState(1);
                }
                break;
            }
            case BTN_NETWORK_DISABLE: {
                if (!getPressRecord(value)) {
                    LOG(INFO) << "Disable network interface";

                    setNetworkInterfaceState(0);
                }
                break;
            }
            case BTN_TIEAN_OPERATION_MODE: {
                LOG(INFO) << "BTN_TIEAN_OPERATION_MODE";
                if (getPressRecord(value)) {
                    return;
                }
                if (is_tiean_manual_control_mode_) {
                    LOG(INFO) << "AUTO CONTROL pressed!";
                    msg1->command = CMD_COMMON_CANCEL;
                    is_msg1_shoud_be_sent = true;
                } else {
                    LOG(INFO) << "MANUAL CONTROL pressed!";
                    makeActionTask(15, 2, 0);
                }
                break;
            }
            default: {
                LOG(ERROR) << "Undefine button!";
            }
        }
    } else {
        if (value == BTN_ACTION_ENABLE) {
            is_action_task_enable_ = false;
        } else if (value == BTN_MTASK_ENABLE) {
            is_move_task_enable_ = false;
        }
    }

    if (is_msg1_shoud_be_sent) {
        LOG(INFO) << "MSG1 sent!";
        sendMsg(msg1);
    }

    if (is_msg2_shoud_be_sent) {
        sendMsg(msg2);
    }

    if (is_msg3_shoud_be_sent) {
        sendMsg(msg3);
    }

    updatePressRecord(value, operation_result);
}

void ScreenModule::updateButtonsState() {
    // request button state
    updateButtonState(BTN_OPERATION_MODE, READ_BYTE);

    updateButtonState(BTN_START_LOC, READ_BYTE);
    updateButtonState(BTN_CANCEL_LOC, READ_BYTE);
    updateButtonState(BTN_SPEAKER, READ_BYTE);
    updateButtonState(BTN_CONTINUE, READ_BYTE);
    updateButtonState(BTN_PAUSE, READ_BYTE);

    updateButtonState(BTN_CANCEL, READ_BYTE);
    updateButtonState(BTN_CLEAR, READ_BYTE);
    updateButtonState(BTN_VOLUME_PLUS, READ_BYTE);
    updateButtonState(BTN_VOLUME_MINU, READ_BYTE);

    updateButtonState(BTN_CONFIRM_GOODS, READ_BYTE);

    updateButtonState(BTN_ENTER_POWER_SAVE_MODE, READ_BYTE);
    updateButtonState(BTN_EXIT_POWER_SAVE_MODE, READ_BYTE);

    updateButtonState(BTN_ACTION_ENABLE, READ_BYTE);
    updateButtonState(BTN_MTASK_ENABLE, READ_BYTE);
    updateButtonState(BTN_EXCUTE_TASK, READ_BYTE);
    updateButtonState(BTN_TASK_CLEAR, READ_BYTE);
    updateButtonState(BTN_CANCEL_EMERGENCY, READ_BYTE);
    updateButtonState(BTN_REBOOT_DEVICE, READ_BYTE);

    updateButtonState(BTN_START_CHARGE1, READ_BYTE);
    updateButtonState(BTN_START_CHARGE2, READ_BYTE);

    updateButtonState(BTN_NETWORK_DISABLE, READ_BYTE);
    updateButtonState(BTN_NETWORK_ENABLE, READ_BYTE);

    updateButtonState(BTN_TIEAN_OPERATION_MODE, READ_BYTE);

    if (g_state.operation_state == OPERATION_MANUAL) {
        if(g_state.isChargeState()){
            LOG(WARNING) << "Charing, can't move!";
            return ;
        }
        auto msg = make_shared<sros::core::CommandMsg>(getName());
        msg->command = sros::core::CMD_SET_SRC_SPEED;
        auto t1 = requestButtonState(BTN_GO_STRAIGHT, 1, READ_BYTE);
        auto t2 = requestButtonState(BTN_GO_BACK, 1, READ_BYTE);
        auto t3 = requestButtonState(BTN_GO_LEFT, 1, READ_BYTE);
        auto t4 = requestButtonState(BTN_GO_RIGHT, 1, READ_BYTE);
        if (t1 && !t2 && !t3 && !t4) {
            msg->param0 = 295;
            sendMsg(msg);
        } else if (!t1 && t2 && !t3 && !t4) {
            msg->param0 = -295;
            sendMsg(msg);
        } else if (!t1 && !t2 && t3 && !t4) {
            msg->param1 = 389;
            sendMsg(msg);
        } else if (!t1 && !t2 && !t3 && t4) {
            msg->param1 = -389;
            sendMsg(msg);
        } else if (t1 && !t2 && t3 && !t4) {
            msg->param0 = 295;
            msg->param1 = 389;
            sendMsg(msg);
        } else if (t1 && !t2 && !t3 && t4) {
            msg->param0 = 295;
            msg->param1 = -389;
            sendMsg(msg);
        } else if (!t1 && t2 && t3 && !t4) {
            msg->param0 = -295;
            msg->param1 = 389;
            sendMsg(msg);
        } else if (!t1 && t2 && !t3 && t4) {
            msg->param0 = -295;
            msg->param1 = -389;
            sendMsg(msg);
        }
    }
}

bool ScreenModule::updateScreenTEXT(uint16_t address, uint16_t value, uint8_t mode) {
    // mode : WRITE_WORD write register word; WRITE_BYTE write bytes
    if (mode != WRITE_BYTE && mode != WRITE_WORD) {
        LOG(ERROR) << "Invalid mode for update Screen text!";
        return false;
    }
    vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(mode);
    data.push_back(address >> 8);
    data.push_back(address);

    if ((value == 0xff || value == 0) && mode == WRITE_BYTE) {
        data.push_back(value);
        data.push_back(value >> 8);
    } else if (mode == WRITE_WORD) {
        data.push_back(value >> 8);
        data.push_back(value);
    }

    auto sum = getCRCChecksum((char *)&data[0], data.size());
    data.push_back(sum);
    data.push_back(sum >> 8);

    std::string data_str = VectorcharToString(data);
    try {
        timeout_serial_->writeString(data_str);
    } catch (const std::exception &e) {
        LOG(ERROR) << "write error! " << e.what();
        return false;
    }
    std::string recv_result;
    try {
        recv_result = timeout_serial_->readString(data.size());
    } catch (...) {
        LOG_EVERY_N(ERROR, 50) << "serial read timeout. address = " << hex << (int)address;
        return false;
    }

    if (recv_result != data_str) {
        LOG(WARNING) << "failed to updateScreenTEXT address " << hex << (int)address;
        return false;
    }

    return true;
}

bool ScreenModule::update32ScreenTEXT(uint16_t address, uint32_t value) {
    vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(0x10);
    data.push_back(address >> 8);
    data.push_back(address);
    data.push_back(0x00);
    data.push_back(0x02);
    data.push_back(0x04);

    data.push_back((value >> 8) & 0xff);
    data.push_back((value >> 0) & 0xff);
    data.push_back((value >> 24) & 0xff);
    data.push_back((value >> 16) & 0xff);

    auto sum = getCRCChecksum((char *)&data[0], data.size());
    data.push_back(sum);
    data.push_back(sum >> 8);

    std::string data_str = VectorcharToString(data);
    timeout_serial_->writeString(data_str);
    std::string recv_result;
    try {
        recv_result = timeout_serial_->readString(data.size());
    } catch (...) {
        LOG(ERROR) << "serial read timeout. address = " << hex << (int)address;
        return false;
    }

    if (recv_result == data_str) {
        return true;
    } else {
        LOG(WARNING) << "failed to updateScreenTEXT address " << hex << (int)address;
        return false;
    }
}

void ScreenModule::updateIPaddressTEXT() {
    if (pre_local_ip_address_ == local_ip_address_) {
        return;
    }
    if (local_ip_address_.empty()) {
        return;
    }
    std::vector<std::string> result = splitWithStl(local_ip_address_, ".");

    if (result.size() == 4) {
        updateScreenTEXT(SYS_IP1_ADDR, atoi(result[0].c_str()), WRITE_WORD);
        updateScreenTEXT(SYS_IP2_ADDR, atoi(result[1].c_str()), WRITE_WORD);
        updateScreenTEXT(SYS_IP3_ADDR, atoi(result[2].c_str()), WRITE_WORD);
        updateScreenTEXT(SYS_IP4_ADDR, atoi(result[3].c_str()), WRITE_WORD);
    }

    pre_local_ip_address_ = local_ip_address_;
}

uint16_t ScreenModule::requestButtonState(uint16_t address, uint16_t value, uint8_t mode) {
    uint16_t result = 0x0000;
    if (value > 1) {
        LOG(WARNING) << "Unspported read method , please check your code!";
        return result;
    }
    vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(mode);
    data.push_back(address >> 8);
    data.push_back(address);

    data.push_back(value >> 8);
    data.push_back(value);

    auto sum = getCRCChecksum((char *)&data[0], data.size());
    data.push_back(sum);
    data.push_back(sum >> 8);

    int len = 0;
    if (mode == READ_BYTE) {
        len = value + 4 + 1;
    } else if (mode == READ_WORD) {
        len = value * 2 + 4 + 1;
    }

    std::string data_str = VectorcharToString(data);

    try {
        timeout_serial_->writeString(data_str);
    } catch (const std::exception &e) {
        LOG(ERROR) << "write error! " << e.what();
        return result;
    }

    std::string recv_result;
    try {
        recv_result = timeout_serial_->readString(len);
    } catch (...) {
        LOG_EVERY_N(ERROR, 30) << "serial read timeout. address = " << hex << (int)address;
        return result;
    }

    if (!checkRecvFrame(data, recv_result)) {
        LOG(ERROR) << "Recv frame check error";
        try {
            timeout_serial_->readString(
                10);  // NOTE: 若出现断包粘包的问题，将缓存中的数据读空，简单的处理，但是否好使，未测试
        } catch (...) {
            LOG(ERROR) << "serial read timeout. address = " << hex << (int)address;
        }
        return false;
    }

    if (mode == READ_BYTE && value == 1) {
        result = recv_result[3] & 0x0001;
    } else if (mode == READ_WORD && value == 1) {
        result = (((uint16_t)recv_result[3]) << 8) + recv_result[4];
    }
    return result;
}

bool ScreenModule::checkRecvFrame(const vector<uint8_t> &data, const std::string &recv) {
    if (data.size() < 8) {
        LOG(ERROR) << "Invalid frame!";
        return false;
    }
    if (recv[0] != data[0] || recv[1] != data[1]) {
        LOG(ERROR) << "HEAD failure!";
        return false;
    }

    auto sum = getCRCChecksum((char *)&recv[0], recv[2] + 3);
    if ((uint8_t)sum != recv[recv[2] + 3] || (sum >> 8) != recv[recv[2] + 4]) {
        LOG(ERROR) << "CRC failure! sum " << hex << (int)sum;
        return false;
    }
    return true;
}

uint16_t ScreenModule::getCRCChecksum(const char *data, int len) {
    unsigned char *buffer = new unsigned char[len];
    if (len && buffer) {
        memcpy(buffer, (char *)&data[0], len * sizeof(unsigned char));
    } else {
        return 0;
    }
    boost::crc_basic<16> crc_ccitt1(0x8005, 0xFFFF, 0, true, true);
    crc_ccitt1.process_bytes(buffer, len);
    delete[] buffer;
    return crc_ccitt1.checksum();
}

std::string ScreenModule::VectorcharToString(const vector<uint8_t> &data) {
    std::string ret;
    for (auto p : data) {
        ret += p;
    }
    return ret;
}

void ScreenModule::printDataStr(const std::string &data) {
    for (int i = 0; i < data.size(); i++) {
        std::cout << " " << std::setfill('0') << std::setw(2) << std::hex << (int)data[i];
    }
    std::cout << std::endl;
}

void ScreenModule::printData(const vector<uint8_t> &data) {
    for (int i = 0; i < data.size(); i++) {
        std::cout << " " << std::setfill('0') << std::setw(2) << std::hex << (int)data[i];
    }
    std::cout << std::endl;
}

std::vector<std::string> ScreenModule::splitWithStl(const std::string &str, const std::string &pattern) {
    std::vector<std::string> resVec;

    if ("" == str) {
        return resVec;
    }

    std::string strs = str + pattern;

    size_t pos = strs.find(pattern);
    size_t size = strs.size();

    while (pos != std::string::npos) {
        std::string x = strs.substr(0, pos);
        resVec.push_back(x);
        strs = strs.substr(pos + 1, size);
        pos = strs.find(pattern);
    }
    return resVec;
}

void ScreenModule::updatePressRecord(uint16_t id, uint16_t result) {
    auto it = press_record_.find(id);
    if (it != press_record_.end()) {
        it->second = (result > 0) ? true : false;
    } else {
        press_record_.insert(make_pair(id, (result > 0) ? true : false));
    }
}

bool ScreenModule::getPressRecord(uint16_t id) {
    auto it = press_record_.find(id);
    if (it != press_record_.end()) {
        return it->second;
    } else {
        return false;
    }
}

void ScreenModule::makeActionTask(int action_p1, int action_p2, int action_p3) {
    auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (cur_task && cur_task->isRunning()) {
        LOG(WARNING) << "ACTION_TASK: previous task is running, new task is ignored.";
        return;
    }

    auto new_task = std::make_shared<sros::core::ActionTask>(0, getName(), action_p1, action_p2, action_p3);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_NEW_ACTION_TASK;
    mm->param0 = new_task->getTaskNo();
    mm->action_task = new_task;

    sendMsg(mm);
}

void ScreenModule::makeMoveTask(int dst_station) {

    if(g_state.isChargeState()){
        LOG(WARNING) << "Charing, can't move!";
        return ;
    }

    std::deque<sros::core::StationNo_t> dst_stations;
    dst_stations.push_back((sros::core::StationNo_t)dst_station);

    auto new_task = std::make_shared<sros::core::MovementTask>(0, getName(), dst_stations, OBSTACLE_AVOID_WAIT);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
    mm->movement_task = new_task;

    sendMsg(mm);
}

void ScreenModule::handleMixTask() {
    if (is_mix_task_running_) {
        return;
    }
    LOG(INFO) << "-- > Mix task thead start!";
    if (is_action_task_enable_ && is_move_task_enable_) {
        if (move_dst_station_ > 0) {
            makeMoveTask(move_dst_station_);
            auto move_task = sros::core::TaskManager::getInstance()->getMovementTask();
            while (move_task && move_task->isRunning()) {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
            }
        }

        makeActionTask(action_p1_, action_p2_, action_p3_);
        auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
        while (action_task && action_task->isRunning()) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
        }
        is_mix_task_running_ = false;
        LOG(INFO) << "-- > Mix task thead exit!";
    }
}

void ScreenModule::cancelActionTask() {
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (action_task && action_task->isRunning()) {
        LOG(WARNING) << "Current actio task will be cancelled!! ";
        action_task->finishTask(sros::core::TASK_RESULT_CANCELED, 0);

        auto notify_msg = std::make_shared<sros::core::NotificationMsg>("TOPIC_NOTIFY");
        notify_msg->notify_type = sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED;
        notify_msg->action_task = action_task;

        sendMsg(notify_msg);
    }
}

void ScreenModule::updateState(sros::core::NavigationState state) {
    auto mm = make_shared<CommonStateMsg<sros::core::NavigationState>>("NAV_STATE");
    mm->state = state;
    sendMsg(mm);
    return;
}

void ScreenModule::getPingMS() {
    try {
        std::string index = "network." + interface_ + "_gateway";
        gateway_ = sros::core::Settings::getInstance().getValue<std::string>(index.c_str(), "192.168.83.1");
        LOG(INFO) << "index = " << index << " gateway = " << gateway_;

        boost::asio::io_service io_service;
        network::icmp::pinger p(io_service, gateway_.c_str());
        p.setPingCallbackFunc(boost::bind(&ScreenModule::updatePingValue, this, _1));
        p.setTimeoutCallbackFunc(boost::bind(&ScreenModule::updatePingTimeout, this));

        io_service.run();
    } catch (std::exception &e) {
        LOG(WARNING) << "Exception: " << e.what();
    }
}

void ScreenModule::updatePingTimeout() {
    ping_ms_ = 4000;
    is_device_online_ = false;
}

void ScreenModule::updatePingValue(double value) {
    ping_ms_ = value;

    if (ping_ms_ < 50 && ping_ms_ >= 0) {
        signal_ = SS_GOOD;
    } else if (ping_ms_ < 250 && ping_ms_ >= 50) {
        signal_ = SS_MIDDLE;
    } else if (ping_ms_ < 1000 && ping_ms_ >= 250) {
        signal_ = SS_POOR;
    } else {
        signal_ = SS_DISCONNECT;
    }
    is_device_online_ = true;
}

void ScreenModule::setNetworkInterfaceState(int state) {
    char command[32];

    std::string format_str;
    if (state > 0) {
        format_str = "ip link set %s up";
    } else {
        format_str = "ip link set %s down";
    }
    int command_len = format_str.size() + interface_.size() + 1 - 2;

    if (command_len > 32) {
        return;
    }

    snprintf(command, command_len, format_str.c_str(), interface_.c_str());

    LOG(INFO) << "network device init command = " << command << "##";

    pid_t status = system(command);
    if (!(status != -1 && WIFEXITED(status) && 0 == WEXITSTATUS(status))) {
        LOG(WARNING) << command << " failed";
        return;
    }

    is_device_online_ = (state > 0) ? true : false;
}

}  // namespace tscreen
