//
// Created by lhx on 16-3-4.
//

#include "state.h"
#include <glog/logging.h>
#include "core/msg/common_msg.hpp"
#include "core/msg_bus.h"
#include "core/settings.h"
#include "core/user_manager.h"
#include "device/device.h"
#include "exec_error.hpp"
#include "fault_center.h"

// 全局变量
sros::core::State g_state;
sros::core::DeviceSRCState g_src_state;

namespace sros {
namespace core {

State::State()
    : main_state(STATE_INITIALING),
      slam_state(STATE_SLAM_IDLE),
      nav_state(STATE_NAV_IDLE),
      sys_state(SYS_STATE_INITIALING),
      location_state(LOCATION_STATE_NONE),
      hardware_state(H_STATE_OK),
      station_no(0),
      progress(0),
      action_state(STATE_ACTION_NA),
      battery_percentage(0),
      battery_current(0),
      battery_voltage(0),
      battery_temperature(0),
      battery_state(BATTERY_NA),
      power_state(POWER_NA),
      break_sw_state(BREAK_SW_NA),
      emergency_state(STATE_EMERGENCY_NA),
      emergency_source(EMERGENCY_SRC_NONE),
      operation_state(OPERATION_AUTO),
      oba_state(OBA_ENABLED),
      fresh_state(FRESH_YES),
      load_state(LOAD_FREE),
      speed_level(100),
      hardware_error_code(0),
      cur_volume(0),
      sros_emergency_src(0){

}

void State::checkStartMovementTaskCondition() const {
    if (emergency_state == sros::core::STATE_EMERGENCY_TRIGER ||
        emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_IN_EMERGENCY, "vehicle is in emergency mode");
    } else if (slam_state != STATE_SLAM_LOCATING) {
        auto &s = sros::core::Settings::getInstance();
        auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
        if (enable_sros_native_debug) {
            return;  // 若是本地调试，就不需要检查这些东西
        }

        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_NO_LOCATION, "vehicle is NOT in locating! slam_state:", slam_state);
    } else if (isManualControl()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_IN_MANUAL_CONTROL, "vehicle is in manual control");
    } else if (isPowerSaveMode()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_IN_POWER_SAVE_MODE, "vehicle is in power save mode");
    } else if (isBreakSwitchON()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_BREAK_SWITCH_ON, "vehicle is break switch on");
    } else if (isLaserError()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_LASER_ERROR, "laser error");
        //    } else if (isVscError()) { //
        //    暂时屏蔽vsc超时，vsc超时属于硬件缺陷，长时间内无法根治，但启动程序的时候有概率vsc刚好超时，是的启动任务失败，从而使mission运行失败。
        // throw EXEC_ERROR(ERROR_CODE_MOVEMENT_VSC_ERROR, "VSC error");
    } else if (isSrcError()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_SRC_ERROR, "SRC error");
    } else if (isMotor1Error()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_MOTOR1_ERROR, "Motor 1 error");
    } else if (isMotor2Error()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_MOTOR2_ERROR, "Motor 2 error");
    } else if (isChargeState()) {
        // 充电过程中不能启动移动任务
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_BATTERY_CHARGING, "Please stop charge first!");
    }
}

// 检测启动动作任务的条件
void State::checkStartActionTaskCondition(const int& _actionId,const int& _param0,const int& _param1) const {
    if (emergency_state == sros::core::STATE_EMERGENCY_TRIGER ||
        emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE) {
        throw EXEC_ERROR(ERROR_CODE_ACTION_IN_EMERGENCY, "vehicle is in emergency mode!");
    } else if (isPowerSaveMode()) {
        throw EXEC_ERROR(ERROR_CODE_ACTION_IN_POWER_SAVE_MODE, "vehicle is in power save mode!");
    } else if (isBreakSwitchON()) {
        throw EXEC_ERROR(ERROR_CODE_ACTION_BREAK_SWITCH_ON, "Please close break switch first!");
    } else if (isManualControl()) {
        throw EXEC_ERROR(ERROR_CODE_MANUAL_CONTROL_RUNNING, "vehicle is in manual control mode!");
    } else if (isChargeState()) {
        // 充电过程中只能响应“开始/停止充电”的动作
        if ((_actionId == 78 && _param0 == 1 && _param1 == 0) ||
            (_actionId == 78 && _param0 == 2 && _param1 == 0)) {
            LOG(INFO) << "Got start/stop charging task while battery is charging";
        } else {
            throw EXEC_ERROR(ERROR_CODE_ACTION_BATTERY_CHARGING, "Please stop charge first!");
        }
    }
}

std::string State::getCurMapName() const {
    boost::shared_lock<boost::shared_mutex> lock(cur_map_name_mutex_);
    return cur_map_name_;
}

void State::setCurMapName(const std::string &map_name) {
    boost::lock_guard<boost::shared_mutex> look(cur_map_name_mutex_);
    cur_map_name_ = map_name;
}

void State::setNeedDebugInfo(bool enable) {
    if (need_debug_info_ != enable) {
        if (enable) {
            // 当开启调试信息模式时，需要开启下视PGV
            auto msg = std::make_shared<sros::core::CommonMsg>("TOPIC_SVC100_ENABLE_PUBLISH");
            msg->flag = true;
            msg->str_0_ = sros::device::DEVICE_SVC100_DOWN;
            MsgBus::getInstance()->sendMsg(msg);
        } else {
            // 当关闭调试信息模式时，且没有运行矫正任务，需要关闭下视PGV
            auto task = TaskManager::getInstance()->getActionTask();
            if (!task || !task->isSlaveRunning() || task->getActionID() != ACTION_ID_PGV_RECTIFY) {
                auto msg = std::make_shared<sros::core::CommonMsg>("TOPIC_SVC100_ENABLE_PUBLISH");
                msg->flag = false;
                msg->str_0_ = sros::device::DEVICE_SVC100_DOWN;
                MsgBus::getInstance()->sendMsg(msg);
            }
        }

        need_debug_info_ = enable;
    }
}

int State::getActionControllerType() const {
    auto &s = sros::core::Settings::getInstance();
    int action_controller_type = s.getValue<int>("src.continuous_mode", ACTION_CONTROLLER_TYPE_SRC_JACKING_ROTATE);

    return action_controller_type;
}

void State::updateFleetState() {
    if (g_state.fleet_mode == FLEET_MODE_OFFLINE) {
        if (g_state.control_mutex.isLock(USERNAME_FMS) || g_state.control_mutex.isLock(USERNAME_SROS)) {
            // 当切换到单机模式时，需要解除fms的独占
            g_state.control_mutex.unlock();
            FaultCenter::getInstance()->removeFault(FAULT_CODE_FMS_DISCONNECT);
        }
    } else if (g_state.fleet_mode == FLEET_MODE_ONLINE) {
        auto item = network_session_manager.getConnectedItem(USERNAME_FMS);
        if (item) {
            g_state.control_mutex.unlock();
            g_state.control_mutex.lock(item->session_id, item->ip_addr, item->username, item->username);
            FaultCenter::getInstance()->removeFault(FAULT_CODE_FMS_DISCONNECT);
        } else {
            g_state.control_mutex.unlock();
            g_state.control_mutex.lock(0, "localhost", USERNAME_SROS, "fleet");
            FaultCenter::getInstance()->addFault(FAULT_CODE_FMS_DISCONNECT);
        }
    }
}

void State::playMusic(hmi::MusicID music_id) {
    if (music_id != hmi::MUSIC_ID_MUSIC_NONE) {
        auto msg = std::make_shared<sros::core::HmiMsg>();
        msg->command = HMI_COMMAND_SET_SUCCEED_MUSIC;
        msg->int_0 = music_id;
        MsgBus::sendMsg(msg);
    }
}

}  // namespace core
}  // namespace sros
