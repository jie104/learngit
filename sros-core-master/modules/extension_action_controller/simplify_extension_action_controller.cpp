//
// Created by caoyan on 6/22/21.
//

#include "simplify_extension_action_controller.h"
#include "core/device/can_interface.h"
#include "core/device/com_port_interface.hpp"
#include "core/device/device_manager.h"
#include "core/exec_error.hpp"
#include "core/fault_center.h"
#include "core/hardware/EAC.h"
#include "core/logger.h"
#include "core/msg/can_data_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/msg/emergency_msg.h"
#include "core/msg/common_state_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/state.h"

using namespace std;
using namespace sros::core;
using namespace sros::modbus;
using namespace sros::device;

namespace sros {
namespace eac {

void SimplifyExtensionActionController::run() {
    LOG(INFO) << "SimplifyExtensionActionController module start running";

    auto &s = core::Settings::getInstance();
    if (s.getValue<std::string>("device.enable_eac", "False") == "False") {
        LOG(INFO) << "SimplifyExtensionActionController module stop running(eac device disable)";
        stop();
        return;
    }

    if (s.getValue<std::string>("device.eac_comm_protocol", "EACP") != "SEACP") {
        LOG(INFO) << "SimplifyExtensionActionController module stop running(eac comm protocol EACP)";
        stop();
        return;
    }

    //参数获取
    auto eac_basis_input_register_addr = s.getValue<int>("device.eac_basis_input_register_addr", 30300);
    auto eac_basis_input_register_num = s.getValue<int>("device.eac_basis_input_registers_num", 50);
    auto eac_basis_hold_register_addr = s.getValue<int>("device.eac_basis_hold_register_addr", 40300);
    auto eac_basis_hold_register_num = s.getValue<int>("device.eac_basis_hold_registers_num", 50);

    seacp_enable_extend_ = (s.getValue<std::string>("device.seacp_enable_extend", "False") == "True");
    auto eac_extend_input_register_addr = s.getValue<int>("device.seacp_extend_input_register_addr", 30350);
    auto eac_extend_input_register_num = s.getValue<int>("device.seacp_extend_input_registers_num", 50);
    auto eac_extend_hold_register_addr = s.getValue<int>("device.seacp_extend_hold_register_addr", 40350);
    auto eac_extend_hold_register_num = s.getValue<int>("device.seacp_extend_hold_registers_num", 50);

    eac_disable_heat_beat_ = (s.getValue<std::string>("device.eac_disable_heat_beat", "False") == "True");
    eac_heat_beat_timeout_ = s.getValue<int>("device.eac_heat_beat_timeout", 5000);

    eac_register_state_keep_time_ = s.getValue<int>("device.eac_register_state_keep_time", 1000);

    auto eac_replace_input_registers_with_hold =
        (s.getValue<std::string>("device.eac_replace_input_registers_with_hold", "False") == "True");

    //连接eac
    synchronizer_.setReplaceInputRegistersWithHoldRegisters(eac_replace_input_registers_with_hold);

    synchronizer_.setBasisInputRegister(driver_.getBasisInputRegistersAddr(), eac_basis_hold_register_addr,
                                        eac_basis_hold_register_num);
    synchronizer_.setBasisHoldRegister(driver_.getBasisHoldRegistersAddr(), eac_basis_input_register_addr,
                                       eac_basis_input_register_num);

    synchronizer_.setEnableExtendFunc(seacp_enable_extend_);
    synchronizer_.setExtendInputRegister(driver_.getExtendInputRegistersAddr(), eac_extend_hold_register_addr,
                                         eac_extend_hold_register_num);
    synchronizer_.setExtendHoldRegister(driver_.getExtendHoldRegistersAddr(), eac_extend_input_register_addr,
                                        eac_extend_input_register_num);

    auto eac_slave_id = s.getValue<int>("device.eac_slave_id", 1);
    synchronizer_.setSlaveId(eac_slave_id);

    if (s.getValue<std::string>("device.eac_communication_type", "modbus_tcp") == "modbus_tcp") {

        std::string eac_ip;
        int eac_port;
        //eac客户定制化功能
        if (s.getValue<std::string>("device.seacp_enable_customize", "False") == "True") {
            LOGGER(INFO, SROS) << "enable seacp customize function.";
            eac_ip = "127.0.0.1";
            eac_port = 503;

        } else {
            eac_ip = s.getValue<std::string>("device.eac_ip", "NA");
            if (eac_ip.empty() || eac_ip == "NA") {
                LOGGER(ERROR, SROS) << "SimplifyExtensionActionController module stop running, eac ip invalid! ip is "
                                    << eac_ip;
                stop();
                return;
            }
            eac_port = s.getValue<int>("device.eac_port", 502);
        }
        synchronizer_.setRemoteTcpInfo(eac_ip, eac_port);
        eac_device_ = createDevice<EAC>(DEVICE_EAC, DEVICE_ID_EAC, DEVICE_COMM_INTERFACE_TYPE_ETH_1, DEVICE_MOUNT_SROS);
    } else {
        auto device_name = s.getValue<std::string>("device.eac_serial_device", "/dev/ttyUSB0");
        if (device_name.empty()) {
            LOGGER(ERROR, SROS) << "SimplifyExtensionActionController module stop running, EAC device name is empty";
            stop();
            return;
        }

        auto enable_touch_screen = (s.getValue<std::string>("hmi.enable_touch_screen", "False") == "True");
        auto touch_screen_device_name = s.getValue<std::string>("hmi.touch_screen_device_name", "/dev/ttyTHS1");
        if (enable_touch_screen && device_name == touch_screen_device_name) {
            LOGGER(ERROR, MODBUS) << "EAC Settings of device_name conflict with touchScreen,"
                                     " SimplifyExtensionActionController will not start!";
            stop();
            return;
        }

        auto enable_lmns_usart = (s.getValue<std::string>("main.enable_lmns_usart", "False") == "True");
        auto lmns_device_name = s.getValue<std::string>("lmns.serial_device", "/dev/ttyTHS1");
        if (enable_lmns_usart && device_name == lmns_device_name) {
            LOGGER(ERROR, MODBUS) << "EAC Settings of device_name conflict with lmns usart,"
                                     " SimplifyExtensionActionController will not start!";
            stop();
            return;
        }

        auto enable_modbus_rtu = (s.getValue<std::string>("modbus.enable_modbus_rtu", "False") == "True");
        auto modbus_device_name = s.getValue<std::string>("modbus.modbus_rtu_device", "/dev/ttyUSB0");
        if (enable_modbus_rtu && device_name == modbus_device_name) {
            LOGGER(ERROR, MODBUS) << "EAC Settings of device_name conflict with modbus rtu,"
                                     " SimplifyExtensionActionController will not start!";
            stop();
            return;
        }

        auto eac_serial_baud_rate = s.getValue<int>("device.eac_serial_baud_rate", 115200);
        synchronizer_.setRemoteRtuInfo(device_name, eac_serial_baud_rate);
        eac_device_ =
            createDevice<VirtualDevice>(DEVICE_EAC, DEVICE_ID_EAC, DEVICE_COMM_INTERFACE_TYPE_USB_1, DEVICE_MOUNT_SROS);
    }

    int insepction_interval = s.getValue<int>("inspection.eac_inspection_intvl", 5000);
    eac_device_->setTimeoutTime(insepction_interval);

    //监听任务
    subscribeTopic("DEBUG_CMD", CALLBACK(&SimplifyExtensionActionController::onDebugCmdMsg));
    subscribeTopic("ACTION_CMD", CALLBACK(&SimplifyExtensionActionController::onActionCmdMsg));
    subscribeTopic("EAC_ACTION_CMD", CALLBACK(&SimplifyExtensionActionController::onEacActionCmdMsg));
    subscribeTopic("TOPIC_EMERGENCY", CALLBACK(&SimplifyExtensionActionController::onEmergencyStateMsg));

    subscribeTopic("TIMER_50MS", CALLBACK(&SimplifyExtensionActionController::onTimer_50ms));
    subscribeTopic("TIMER_1S", CALLBACK(&SimplifyExtensionActionController::onTimer_1s));

    LOG(INFO) << "SimplifyExtensionActionController module started!";

    dispatch();
}

void SimplifyExtensionActionController::onDebugCmdMsg(sros::core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    auto session_id = mm->session_id;
    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            auto locker = g_state.control_mutex.getLocker();
            throw EXEC_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", locker.session_id, "current:", session_id);
        }
    };

    try {
        switch (mm->command) {
            case core::CMD_SRC_RESET: {
                getControlMutexFun();
                driver_.resetEac();
                LOGGER(INFO, CMD_HANDER) << "send reset request to eac!";
                return;
            }
            case CMD_RESET_FAULT: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_RESET_FAULT command";
                getControlMutexFun();

                uint16_t eac_sys_error_code;
                EacSysState eac_sys_state = driver_.getEacSysState(eac_sys_error_code);
                uint32_t eac_act_result, run_action_no;
                EacActState eac_act_state = driver_.getEacActState(eac_act_result, run_action_no);

                if (eac_sys_state == ESS_ERROR) {
                    if (eac_act_state == EAS_ABNORMAL_PAUSING) {
                        driver_.continueAction();
                    } else {
                        driver_.resetEac();
                    }
                }

                return;
            }
            default: {
                return;
                break;
            }
        }
    } catch (const ExecError &e) {
        // 命令处理出错的情况
        mm->result_state = RESPONSE_FAILED;
        mm->result_code = e.errorCode();
        responseCommand(mm);
        return;
    }
}

void SimplifyExtensionActionController::onEacActionCmdMsg(core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<core::CommandMsg>(msg);
    auto session_id = mm->session_id;

    try {
        switch (mm->command) {
            case core::CMD_NEW_ACTION_TASK: {
                cur_eac_task_ = mm->action_task;
                if (!cur_eac_task_) {
                    LOGGER(WARNING, CMD_HANDER) << "onDebugCmdMsg: action task is NULL";
                    return;  // action_controller_module中回复了，此处不需要回复
                }

                auto action_id = cur_eac_task_->getActionID();
                auto action_no = static_cast<uint32_t>(mm->param0);

                if ((action_id >= 0xC0 && action_id <= 0xFF)) {  // 扩展动作id，用户开发的控制器，定制的动作

                    uint16_t eac_sys_error_code;
                    EacSysState eac_sys_state = driver_.getEacSysState(eac_sys_error_code);


                    //检测eac状态
                    if (eac_sys_state == ESS_ERROR) {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_IN_ERROR_STATE, "Eac Sys current state is",
                                               eac_sys_state, "error code is", eac_sys_error_code);
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_IN_ERROR_STATE, "Eac Sys current state is",
                                         eac_sys_state, "error code is", eac_sys_error_code);

                    } else if (eac_sys_state != ESS_NORMAL) {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_NOT_IDLE, "Eac Sys current state is",
                                               eac_sys_state);
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_NOT_IDLE, "Eac Sys current state is", eac_sys_state);
                    }

                    uint32_t eac_act_result, run_action_no;
                    EacActState eac_act_state = driver_.getEacActState(eac_act_result, run_action_no);
                    if (eac_act_state != EAS_IDLE) {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_NOT_IDLE, "Eac Act current state is",
                                               eac_act_state);
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_NOT_IDLE, "Eac Act current state is", eac_act_state);
                    }

                    //检测sros状态
                    EacSysControl eac_sys_control;
                    EacActControl eac_act_control;
                    driver_.getEacParamControl(eac_sys_control, eac_act_control);
                    if (eac_sys_control != ESC_NONE || eac_act_control != EAC_NONE) {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_NOT_IDLE, "eac_sys_control current state is",
                                               eac_sys_control, "eac_act_state current state is", eac_act_state);
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_NOT_IDLE, "eac_sys_control current state is",
                                         eac_sys_control, "eac_act_state current state is", eac_act_state);
                    }

                    //启动任务
                    auto action_param0 = cur_eac_task_->getActionParam();
                    auto action_param1 = cur_eac_task_->getActionParam1();

                    driver_.newAction(action_no, action_id, action_param0, action_param1);

                } else {
                    return;
                }
                break;
            }
            case core::CMD_CANCEL_ACTION_TASK: {
                auto action_task = core::TaskManager::getInstance()->getActionTask();
                if (!action_task) {
                    LOGGER(WARNING, CMD_HANDER) << "onDebugCmdMsg: action task is NULL";
                    return;
                }

                if ((action_task->isSlaveRunning() || action_task->isWaitForStart()) && cur_eac_task_) {
                    int action_no = cur_eac_task_->getTaskNo();
                    LOG(INFO) << "cancel eac action task, action no: " << action_no;
                    if (action_no == mm->param0) {
                        cur_eac_task_ = nullptr;
                        driver_.cancelAction();
                        TaskManager::getInstance()->setActionFinishCanceled();
                    }
                }
                return;
                break;
            }
            default: {
                return;
                break;
            }
        }
    } catch (const ExecError &e) {
        auto action_task = core::TaskManager::getInstance()->getActionTask();
        if (action_task) {
            ActionStatus action_status;
            action_status.action_no_ = action_task->getTaskNo();
            action_status.action_id_ = action_task->getActionID();
            action_status.action_result_ = TASK_RESULT_FAILED;
            action_status.action_result_value_ = e.errorCode();
            responseEacActionResult(action_status);
        }
        return;
    }
}

void SimplifyExtensionActionController::onActionCmdMsg(core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<core::CommandMsg>(msg);

    switch (mm->command) {
        case core::CMD_SRC_PAUSE: {
            auto action_task = core::TaskManager::getInstance()->getActionTask();
            if (!action_task) {
                LOGGER(WARNING, CMD_HANDER) << "onDebugCmdMsg: action task is NULL";
                return;
            }

            if ((action_task->isSlaveRunning() || action_task->isWaitForStart()) && cur_eac_task_) {
                int action_id = cur_eac_task_->getActionID();
                if ((action_id >= 0xC0 && action_id <= 0xFF)) {  // 扩展动作id，用户开发的控制器，定制的动作
                    driver_.pauseAction();
                    mm->result_state = RESPONSE_OK;
                    break;
                }
            }
            return;
        }
        case core::CMD_SRC_CONTINUE: {
            auto action_task = core::TaskManager::getInstance()->getActionTask();
            if (!action_task) {
                LOGGER(WARNING, CMD_HANDER) << "onDebugCmdMsg: action task is NULL";
                return;
            }

            if ((action_task->isSlaveRunning() || action_task->isWaitForStart()) && cur_eac_task_) {
                int action_id = cur_eac_task_->getActionID();
                if ((action_id >= 0xC0 && action_id <= 0xFF)) {  // 扩展动作id，用户开发的控制器，定制的动作
                    driver_.continueAction();
                    mm->result_state = RESPONSE_OK;
                    break;
                }
            }
            return;
        } 
        default: {
            return;
        }
    }
    responseCommand(mm);
}

void SimplifyExtensionActionController::onEmergencyStateMsg(core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<core::EmergencyMsg>(msg);
    if (mm->emergency_state == STATE_EMERGENCY_TRIGER || mm->emergency_state == STATE_EMERGENCY_RECOVERABLE) {
        driver_.pauseAction();

    } else {
        uint16_t eac_sys_error_code;
        EacSysState eac_sys_state = driver_.getEacSysState(eac_sys_error_code);
        uint32_t eac_act_result, run_action_no;
        EacActState eac_act_state = driver_.getEacActState(eac_act_result, run_action_no);

        if (eac_sys_state == ESS_ERROR) {
            if (eac_act_state == EAS_ABNORMAL_PAUSING) {
                driver_.continueAction();
            } else {
                driver_.resetEac();
            }
        }
    }
}

void SimplifyExtensionActionController::updateHeatBeat() {

    auto updateEacSysState = [&]() {
          //更新系统状态
          uint16_t eac_sys_error_code;
          EacSysState eac_sys_state = driver_.getEacSysState(eac_sys_error_code);
          switch (eac_sys_state) {
              case ESS_NONE: {
                  eac_device_->setStateNone();
                  break;
              }
              case ESS_INITING: {
                  eac_device_->setStateInitialization();
                  break;
              }
              case ESS_ERROR: {
                  eac_device_->setStateError(eac_sys_error_code);
                  break;
              }
              case ESS_NORMAL: {
                  eac_device_->setStateOK();
                  break;
              }
              default: {
                  LOG(WARNING) << "unsupport eac sys state: " << eac_sys_state;
                  break;
              }
          }

        eac_device_->setVersionNo(driver_.getEacVersion());
    };

    //心跳处理
    if (eac_disable_heat_beat_) {
        eac_device_->keepAlive();
        updateEacSysState();

    } else {

        static uint16_t refresh_count = 0;

        if ( refresh_count * 50 >= eac_heat_beat_timeout_) { // 50ms 一次
            static uint16_t old_heat_beat = 0;
            auto heat_beat = driver_.getEacHeartbeat();

            if (old_heat_beat != heat_beat) {
                eac_device_->keepAlive();

                updateEacSysState();

                old_heat_beat = heat_beat;
            }

            refresh_count = 0;
        }

        refresh_count = refresh_count + 1;
    }
}

void SimplifyExtensionActionController::onTimer_1s(core::base_msg_ptr msg) {
    if (g_state.sys_state == SYS_STATE_IDLE && !synchronizer_.isRunning()) {
        synchronizer_.run();  // 待系统启动完成后，才让其启动，否则很有可能将一些不正确的数据传过去了
    }
}

void SimplifyExtensionActionController::onTimer_50ms(core::base_msg_ptr msg) {

    if (!synchronizer_.connected() || !synchronizer_.synchronizer_ok()) {
        eac_device_->setStateTimeout();
        return;
    }

    //心跳处理
    updateHeatBeat();

    //设置负载状态
    updateLoadState();

    //静止移动
    g_state.eac_prohibit_movement_task = driver_.isBanMoveTask();

    //更新eac动作状态
    updateEacActionTaskState();

    //更新sros动作状态
    updateSrosActionTaskState();

    //EacSysControl 和 EacActControl 状态重置
    monitorControlState();
}

void SimplifyExtensionActionController::monitorControlState() {
    //检测control状态
    EacSysControl eac_sys_control;
    EacActControl eac_act_control;
    driver_.getEacParamControl(eac_sys_control, eac_act_control);
    uint64_t cur_time = util::get_time_in_ms();

    uint16_t eac_sys_error_code;
    EacSysState eac_sys_state = driver_.getEacSysState(eac_sys_error_code);
    uint32_t eac_act_result, run_action_no;
    EacActState eac_act_state = driver_.getEacActState(eac_act_result, run_action_no);

    //eac_register_state_keep_time_
    //EacSysControl
    static EacSysControl last_eac_sys_control = ESC_NONE;

    static uint64_t eac_sys_control_deal = ESC_NONE;
    static uint64_t eac_sys_control_deal_time = util::get_time_in_ms();

    //LOG(INFO) << "-> cur_eac_sys_control: " << eac_sys_control << ", last_eac_sys_control: " << last_eac_sys_control;

    if (eac_sys_control != last_eac_sys_control) {

        if (eac_sys_control >= ESC_RESET && eac_sys_control <= ESC_CLEAR) {
            eac_sys_control_deal = eac_sys_control;
            eac_sys_control_deal_time = util::get_time_in_ms();
        } else {
            eac_sys_control_deal = ESC_NONE;
        }
    }

    //LOG(INFO) << "-> eac_sys_control_deal: " << eac_sys_control_deal;

    if (eac_sys_control_deal == ESC_RESET) {
        if (eac_sys_control_deal_time + eac_register_state_keep_time_ <  cur_time) {
            LOG(INFO) << "sys_time: " << eac_sys_control_deal_time + eac_register_state_keep_time_ << " >= " << cur_time;
            LOG(INFO) << "driver_.resetEacSysControl() ESC_RESET";
            driver_.resetEacSysControl();
            eac_sys_control_deal = ESC_NONE;
        } else if (eac_sys_state == ESS_NORMAL) {
            LOG(INFO) << "driver_.resetEacSysControl() ESS_NORMAL";
            driver_.resetEacSysControl();
            eac_sys_control_deal = ESC_NONE;
        }
    } else if (eac_sys_control_deal == ESC_CLEAR) {
        if (eac_sys_control_deal_time + eac_register_state_keep_time_ <  cur_time) {
            LOG(INFO) << "driver_.resetEacSysControl() ESC_CLEAR";
            driver_.resetEacSysControl();
            eac_sys_control_deal = ESC_NONE;
        } else if (eac_act_state == EAS_IDLE) {
            LOG(INFO) << "driver_.resetEacSysControl() EAS_IDLE";
            driver_.resetEacSysControl();
            eac_sys_control_deal = ESC_NONE;
        }
    }

    last_eac_sys_control = eac_sys_control;


    //EacActControl
    static EacActControl last_eac_act_control = EAC_NONE;

    static uint64_t eac_act_control_deal = ESC_NONE;
    static uint64_t eac_act_control_deal_time = util::get_time_in_ms();

    //LOG(INFO) << "-->> cur_eac_act_control: " << eac_act_control << ", last_eac_act_control: " << last_eac_act_control;

    if (eac_act_control != last_eac_act_control) {

        if ( eac_act_control >= EAC_START_ACTION
            && eac_act_control <= EAC_CANCEL_ACTION) {
            eac_act_control_deal = eac_act_control;
            eac_act_control_deal_time = util::get_time_in_ms();
        } else {
            eac_act_control_deal = ESC_NONE;
        }
    }

    //LOG(INFO) << "-->> eac_act_control_deal: " << eac_act_control_deal;

    switch (eac_act_control_deal) {
        case EAC_START_ACTION:
        case EAC_PAUSE_ACTION:
        case EAC_CONTINUE_ACTION:
        case EAC_CANCEL_ACTION: {
            if (eac_act_control_deal_time + eac_register_state_keep_time_ <  cur_time) {
                LOG(INFO) << "act_time: " << eac_act_control_deal_time + eac_register_state_keep_time_ << " > " << cur_time;
                LOG(INFO) << "driver_.resetEacActControl()";
                driver_.resetEacActControl();
                eac_act_control_deal_time = EAC_NONE;
            }
            break;
        }
    }

    last_eac_act_control = eac_act_control;

}

void SimplifyExtensionActionController::updateLoadState() {
    if (g_state.action_unit == sros::core::ACTION_UNIT_EAC) {
        auto multi_load_state = driver_.getEacLoadState();
        if (g_state.multi_load_state != multi_load_state) {
            g_state.multi_load_state = multi_load_state;
            LOG(INFO) << "multi_load_state: " << g_state.multi_load_state;
        }
        sros::core::LoadState new_load_state = multi_load_state == 0 ? sros::core::LOAD_FREE : sros::core::LOAD_FULL;
        if (g_state.load_state != new_load_state) {
            g_state.load_state = new_load_state;

            if (new_load_state == sros::core::LOAD_FULL) {
                LOG(INFO) << "Load state changed: LOAD_FREE -> LOAD_FULL";
            } else {
                LOG(INFO) << "Load state changed: LOAD_FULL -> LOAD_FREE";
            }
        }
    }
}

void SimplifyExtensionActionController::updateEacActionTaskState() {
    auto action_task = core::TaskManager::getInstance()->getActionTask();
    if (action_task && action_task->isRunning() && !action_task->isWaitForStart() && cur_eac_task_) 
    {
        // 扩展动作id，用户开发的控制器，定制的动作
        int action_id = cur_eac_task_->getActionID();
        if (!(action_id >= 0xC0 && action_id <= 0xFF)) {
            return;
        }
        
        uint32_t eac_act_result, run_action_no;
        EacActState eac_act_state = driver_.getEacActState(eac_act_result, run_action_no);
        if (run_action_no != cur_eac_task_->getTaskNo()) {
            LOG(INFO) << "action no from sros and eac : " << cur_eac_task_->getTaskNo() << "," << run_action_no;
            return;
        }

        ActionStatus action_status;
        action_status.action_no_ = cur_eac_task_->getTaskNo();
        action_status.action_result_value_ = eac_act_result;
        switch (eac_act_state) {
            case EAS_NONE:
            case EAS_IDLE:
                break;
            case EAS_RUNNING: {
                TaskManager::getInstance()->updateActionState(TASK_RUNNING);
                break;
            }
            case EAS_PAUSING:
            case EAS_ABNORMAL_PAUSING: {
                TaskManager::getInstance()->updateActionState(TASK_PAUSED);
                break;
            }
            case EAS_FIN_FAIL: {
                driver_.clearEac();
                action_status.action_result_ = TASK_RESULT_FAILED;
                responseEacActionResult(action_status);
                break;
            }
            case EAS_FIN_CANCEL: {
                driver_.clearEac();
                action_status.action_result_ = TASK_RESULT_CANCELED;
                responseEacActionResult(action_status);
                break;
            }
            case EAS_FIN_SUCCESS: {
                driver_.clearEac();
                action_status.action_result_ = TASK_RESULT_OK;
                responseEacActionResult(action_status);
                break;
            }
            default: {
                LOGGER(ERROR, SROS) << "Get wrong action state: " << eac_act_state
                                    << ", please check EAC ActionState.";
                break;
            }
        }
    }
}

void SimplifyExtensionActionController::updateSrosActionTaskState() {

    uint16_t eac_sys_error_code;
    EacSysState eac_sys_state = driver_.getEacSysState(eac_sys_error_code);

    if (eac_sys_state != ESS_NORMAL) {
        return;
    }

    //处理sys cmd
    static SrosSysControl last_sros_sys_control = SSC_NONE;
    SrosSysControl sros_sys_control = driver_.getSrosSysControl();

    if (sros_sys_control == SSC_RESET && sros_sys_control != last_sros_sys_control) {
        LOG(INFO) << "setStSrosActRsp SAS_NONE";
        driver_.setStSrosActRsp(SAS_NONE, 0x00, 0x00);
    }

    last_sros_sys_control = sros_sys_control;


    //处理act cmd
    static SrosActControl last_sros_act_control = SAC_NONE;
    StSrosActCmd st_sros_act_cmd;
    driver_.getStSrosActCmd(st_sros_act_cmd);

    if (st_sros_act_cmd.sros_act_control_ != last_sros_act_control) {
        auto action_task = std::atomic_load(&sros_sub_task_);
        switch (st_sros_act_cmd.sros_act_control_) {
            case SAC_NONE: {
                break;
            }
            case SAC_PAUSE_ACTION: {
                LOG(ERROR) << "The pause function has not yet implemented!";
                break;
            }
            case SAC_CONTINUE_ACTION: {
                LOG(ERROR) << "The continue function has not yet implemented!";
                break;
            }
            case SAC_CANCEL_ACTION: {
                if (!action_task || !action_task->isRunning()) {
                    LOG(ERROR) << "SROS sub action not running! can not cancel!";
                    return;
                } else {
                    src_sdk->cancelAction(action_task->getTaskNo());
                    action_task->finishTask(TASK_RESULT_CANCELED, 0);
                    std::atomic_store(&sros_sub_task_, action_task);
                    LOG(INFO) << "setStSrosActRsp SAS_FIN_CANCEL";
                    driver_.setStSrosActRsp(SAS_FIN_CANCEL, 0x00, 0x00);
                }
                break;
            }
            case SAC_START_ACTION: {
                if (action_task && !action_task->isFinished()) {
                    LOG(ERROR) << "SROS sub action still in running! can start again!";
                    return;
                } else {
                    if (st_sros_act_cmd.action_id_ != 40) {
                        LOG(ERROR) << "SROS sub action id (" << st_sros_act_cmd.action_id_ << ") not support! action id 40 only support.";
                        return;
                    }

                    action_task = std::make_shared<sros::core::ActionTask>(
                                    st_sros_act_cmd.action_no_, getName(),
                                    st_sros_act_cmd.action_id_, st_sros_act_cmd.action_param0_,
                                    st_sros_act_cmd.action_param1_, st_sros_act_cmd.action_param2_);
                    action_task->updateState(TASK_RUNNING);
                    std::atomic_store(&sros_sub_task_, action_task);
                    LOG(INFO) << "setStSrosActRsp SAS_RUNNING";
                    driver_.setStSrosActRsp(SAS_RUNNING, 0x00, 0x00);
                    // 绑定SRCActionState回调
                    src_sdk->setActionStateCallback(
                        boost::bind(&SimplifyExtensionActionController::onSRCActionState, this, _1));
                    src_sdk->executeActionInt(action_task->getTaskNo(), st_sros_act_cmd.action_id_,
                                           st_sros_act_cmd.action_param0_, st_sros_act_cmd.action_param1_,
                                           (int32_t)(int16_t)st_sros_act_cmd.action_param2_);
                    LOG(INFO) << "######## SRC::executeAction(): "
                              << "no " << action_task->getTaskNo() << ", "
                              << "id " << st_sros_act_cmd.action_id_ << ", "
                              << "p0 " << (int16_t)st_sros_act_cmd.action_param0_ << ", "
                              << "p1 " << (int16_t)st_sros_act_cmd.action_param1_ << ", "
                              << "p2 " << (int16_t)st_sros_act_cmd.action_param2_;
                }
                break;
            }
            default: {
                LOG(ERROR) << "unsupport sros act control: " << st_sros_act_cmd.sros_act_control_;
                break;
            }
        }
    }
    last_sros_act_control = st_sros_act_cmd.sros_act_control_;


}

void SimplifyExtensionActionController::responseCommand(const core::CommandMsg_ptr &msg) {
    auto response_msg = std::make_shared<CommandMsg>(msg->source, "TOPIC_CMD_RESPONSE");
    response_msg->command = msg->command;
    response_msg->session_id = msg->session_id;
    response_msg->req_seq = msg->req_seq;
    response_msg->result_state = msg->result_state;
    response_msg->result_code = msg->result_code;
    LOG(INFO) << "Response command : command = " << response_msg->command
              << " result state = " << response_msg->result_state << " result code = " << response_msg->result_code
              << " seq = " << response_msg->req_seq << " session id = " << response_msg->session_id;
    sendMsg(response_msg);
}

void SimplifyExtensionActionController::responseEacActionResult(int action_no, TaskResult act_task_state, int result_code) 
{
    cur_eac_task_ = nullptr;
    auto response_msg = std::make_shared<CommonStateMsg<TaskResult>>("EAC_ACTION_RESULT");
    response_msg->state = act_task_state;
    response_msg->param_int = action_no;
    response_msg->failed_code_ = result_code;
    LOG(INFO) << "EAC_ACTION_RESULT state = " << (int)response_msg->state << " result code = " << response_msg->failed_code_;
    sendMsg(response_msg);
}

// 回复EAC结果状态
void SimplifyExtensionActionController::responseEacActionResult(const sros::modbus::ActionStatus& _status)
{
    cur_eac_task_ = nullptr;
    auto response_msg = std::make_shared<CommonStateMsg<TaskResult>>("EAC_ACTION_RESULT");
    response_msg->state = _status.action_result_;
    response_msg->param_int = _status.action_no_;
    response_msg->failed_code_ = _status.action_result_value_;
    response_msg->str_reserved = _status.str_reserved;
    LOG(INFO) << "response eac result : " 
        << (int)response_msg->state << "," 
        << response_msg->failed_code_ << ","
        << response_msg->str_reserved;
    sendMsg(response_msg);
}

void SimplifyExtensionActionController::onSRCActionState(sros::core::TaskStateStruct src_action_state) {
    // 在这里更新ActionTask的状态
    const auto &action_no = src_action_state.no_;

    auto action_task = std::atomic_load(&sros_sub_task_);

    if (!action_task || !action_task->isSlaveRunning() || action_task->getTaskNo() != action_no) {
        return;
    }

    switch (src_action_state.state_) {
        case TASK_RUNNING: {
            // 动作正在执行中
            LOG(INFO) << "action " << action_no << " is running ";
            LOG(INFO) << "setStSrosActRsp SAS_RUNNING";
            driver_.setStSrosActRsp(SAS_RUNNING, 0x00, action_no);
            break;
        }
        case TASK_WAIT_FOR_START:
        case TASK_PAUSED:
        case TASK_IN_CANCEL: {
            break;
        }
        case TASK_FINISHED: {
            const auto &action_result = (int32_t)src_action_state.result_;
            const auto &action_result_value = (int32_t)src_action_state.result_value_;
            LOG(INFO) << "=====> TASK: no " << action_no << " finished with value = " << action_result_value
                      << " action_result = " << action_result;
            switch (src_action_state.result_) {
                case TASK_RESULT_OK: {
                    LOG(INFO) << "SRC_ACTION_RESULT_OK";

                    action_task->finishTask(TASK_RESULT_OK, action_result_value);
                    LOG(INFO) << "setStSrosActRsp SAS_FIN_SUCCESS";
                    driver_.setStSrosActRsp(SAS_FIN_SUCCESS, action_result_value, action_no);
                    break;
                }
                case TASK_RESULT_CANCELED: {
                    LOG(INFO) << "SRC_ACTION_RESULT_CANCELED";
                    action_task->finishTask(TASK_RESULT_CANCELED, action_result_value);
                    LOG(INFO) << "setStSrosActRsp SAS_FIN_CANCEL";
                    driver_.setStSrosActRsp(SAS_FIN_CANCEL, action_result_value, action_no);
                    break;
                }
                case TASK_RESULT_FAILED: {
                    LOG(INFO) << "SRC_ACTION_RESULT_FAILED";
                    action_task->finishTask(TASK_RESULT_FAILED, action_result_value);
                    LOG(INFO) << "setStSrosActRsp SAS_FIN_FAIL";
                    driver_.setStSrosActRsp(SAS_FIN_FAIL, action_result_value, action_no);
                    break;
                }
                default: {
                    // 接收到了非法的result值
                    LOG(ERROR) << "Wrong src action result " << action_result;
                    action_task->finishTask(TASK_RESULT_FAILED, action_result_value);
                    LOG(INFO) << "setStSrosActRsp SAS_FIN_FAIL";
                    driver_.setStSrosActRsp(SAS_FIN_FAIL, action_result_value, action_no);
                    break;
                }
            }
            std::atomic_store(&sros_sub_task_, action_task);
            break;
        }
        default: {
            LOG(ERROR) << "Wrong src action result state:  " << src_action_state.state_;
            break;
        }
    }
}

}  // namespace eac
}  // namespace sros