//
// Created by john on 19-5-14.
//

#include "extension_action_controller.h"
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

void ExtensionActionController::run() {
    LOG(INFO) << "ExtensionActionController module start running";

    auto &s = core::Settings::getInstance();
    if (s.getValue<std::string>("device.enable_eac", "False") == "False") {
        LOG(INFO) << "ExtensionActionController module stop running(eac device disable)";
        stop();
        return;
    }

    if (s.getValue<std::string>("device.eac_comm_protocol", "EACP") != "EACP") {
        LOG(INFO) << "ExtensionActionController module stop running(eac comm protocol SEACP)";
        stop();
        return;
    }

    auto eac_start_input_register_addr = s.getValue<int>("device.eac_basis_input_register_addr", 30300);
    auto eac_start_hold_register_addr = s.getValue<int>("device.eac_basis_hold_register_addr", 40300);
    auto eac_slave_id = s.getValue<int>("device.eac_slave_id", 1);
    auto eac_replace_input_registers_with_hold =
        (s.getValue<std::string>("device.eac_replace_input_registers_with_hold", "False") == "True");

    eac_disable_heat_beat_ = (s.getValue<std::string>("device.eac_disable_heat_beat", "False") == "True");
    auto eac_input_register_num = s.getValue<int>("device.eac_basis_input_registers_num", 50);
    auto eac_hold_register_num = s.getValue<int>("device.eac_basis_hold_registers_num", 50);

    synchronizer_.setReplaceInputRegistersWithHoldRegisters(eac_replace_input_registers_with_hold);

    synchronizer_.setBasisInputRegister(30300, eac_start_hold_register_addr, eac_hold_register_num);
    synchronizer_.setBasisHoldRegister(40300, eac_start_input_register_addr, eac_input_register_num);

    synchronizer_.setEnableExtendFunc(false);

    synchronizer_.setSlaveId(eac_slave_id);

    strict_model_ = (s.getValue<std::string>("device.eac_strict_model", "True") == "True");

    if (s.getValue<std::string>("device.eac_communication_type", "modbus_tcp") == "modbus_tcp") {
        auto eac_ip = s.getValue<std::string>("device.eac_ip", "NA");
        if (eac_ip.empty() || eac_ip == "NA") {
            LOGGER(ERROR, SROS) << "ExtensionActionController module stop running, eac ip invalid! ip is " << eac_ip;
            stop();
            return;
        }

        auto eac_port = s.getValue<int>("device.eac_port", 502);

        synchronizer_.setRemoteTcpInfo(eac_ip, eac_port);
        eac_device_ = createDevice<EAC>(DEVICE_EAC, DEVICE_ID_EAC, DEVICE_COMM_INTERFACE_TYPE_ETH_1, DEVICE_MOUNT_SROS);
    } else {
        auto device_name = s.getValue<std::string>("device.eac_serial_device", "/dev/ttyUSB0");
        if (device_name.empty()) {
            LOGGER(ERROR, SROS) << "ExtensionActionController module stop running, EAC device name is empty";
            stop();
            return;
        }

        auto enable_touch_screen = (s.getValue<std::string>("hmi.enable_touch_screen", "False") == "True");
        auto touch_screen_device_name = s.getValue<std::string>("hmi.touch_screen_device_name", "/dev/ttyTHS1");
        if (enable_touch_screen && device_name == touch_screen_device_name) {
            LOGGER(ERROR, MODBUS) << "EAC Settings of device_name conflict with touchScreen,"
                                     " ExtensionActionController will not start!";
            stop();
            return;
        }

        auto enable_lmns_usart = (s.getValue<std::string>("main.enable_lmns_usart", "False") == "True");
        auto lmns_device_name = s.getValue<std::string>("lmns.serial_device", "/dev/ttyTHS1");
        if (enable_lmns_usart && device_name == lmns_device_name) {
            LOGGER(ERROR, MODBUS) << "EAC Settings of device_name conflict with lmns usart,"
                                     " ExtensionActionController will not start!";
            stop();
            return;
        }

        auto enable_modbus_rtu = (s.getValue<std::string>("modbus.enable_modbus_rtu", "False") == "True");
        auto modbus_device_name = s.getValue<std::string>("modbus.modbus_rtu_device", "/dev/ttyUSB0");
        if (enable_modbus_rtu && device_name == modbus_device_name) {
            LOGGER(ERROR, MODBUS) << "EAC Settings of device_name conflict with modbus rtu,"
                                     " ExtensionActionController will not start!";
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

    subscribeTopic("DEBUG_CMD", CALLBACK(&ExtensionActionController::onDebugCmdMsg));
    subscribeTopic("ACTION_CMD", CALLBACK(&ExtensionActionController::onActionCmdMsg));
    subscribeTopic("EAC_ACTION_CMD", CALLBACK(&ExtensionActionController::onEacActionCmdMsg));
    subscribeTopic("TOPIC_EMERGENCY", CALLBACK(&ExtensionActionController::onEmergencyStateMsg));

    subscribeTopic("TIMER_100MS", CALLBACK(&ExtensionActionController::onTimer_100ms));
    subscribeTopic("TIMER_1S", CALLBACK(&ExtensionActionController::onTimer_1s));

    LOG(INFO) << "ExtensionActionController module started!";

    dispatch();
}

void ExtensionActionController::onDebugCmdMsg(sros::core::base_msg_ptr msg) {
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
                driver_.reset();
                LOGGER(INFO, CMD_HANDER) << "send reset request to eac!";
                return;
            }
            case CMD_RESET_FAULT: {
                // FIXME(pengjiali): 废弃，直接在本类设置EAC故障处理办法
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_RESET_FAULT command";
                getControlMutexFun();

                int error_no;
                eac_device_system_state_ = driver_.getSystemState(error_no);
                if (eac_device_system_state_ == SYSTEM_STATE_ERROR) {
                    driver_.reset();
                }
                break;
            }
            case sros::core::CMD_NEW_ACTION_TASK:
            case sros::core::CMD_CANCEL_ACTION_TASK: {
                break;
            }
            default: {
                return;
                break;
            }
        }
    } catch (const ExecError &e) {
        // 命令处理出错的情况
        mm->result_state = RESPONSE_FAILED;
        int error_no;
        eac_device_system_state_ = driver_.getSystemState(error_no);
        mm->result_code = e.errorCode();
        responseCommand(mm);
        return;
    }
}

void ExtensionActionController::onEacActionCmdMsg(core::base_msg_ptr msg) {
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
                    int error_no;
                    eac_device_system_state_ = driver_.getSystemState(error_no);

                    if (eac_device_system_state_ == SYSTEM_STATE_ERROR) {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_IN_ERROR_STATE, "Eac current state is",
                                               eac_device_system_state_, "error code is",
                                               eac_device_->getRawFaultCode());
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_IN_ERROR_STATE, "Eac current state is",
                                         eac_device_system_state_, "error code is", eac_device_->getRawFaultCode());
                    } else if (eac_device_system_state_ != SYSTEM_STATE_IDLE) {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_NOT_IDLE, "Eac current state is",
                                               eac_device_system_state_);
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_NOT_IDLE, "Eac current state is",
                                         eac_device_system_state_);
                    }

                    auto action_param0 = cur_eac_task_->getActionParam();
                    auto action_param1 = cur_eac_task_->getActionParam1();

                    NewActionResult result = NAR_NONE;
                    bool ret = driver_.newAction(action_no, action_id, action_param0, action_param1, result);
                    if (!ret) {
                        switch (result) {
                            case NAR_NONE: {
                                SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_RESPONSE_INCORRECT, "");
                                throw EXEC_ERROR(ERROR_CODE_ACTION_RESPONSE_INCORRECT, "");
                                break;
                            }
                            case NAR_SUCCEED: {
                                mm->result_state = RESPONSE_OK;
                                break;
                            }
                            case NAR_SYSTEM_BUSY: {
                                SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_SYSTEM_BUSY, "");
                                throw EXEC_ERROR(ERROR_CODE_ACTION_SYSTEM_BUSY, "");
                                break;
                            }
                            case NAR_ACTION_ID_NOT_SUPPORT: {
                                SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_ID_NOT_SUPPORT, "");
                                throw EXEC_ERROR(ERROR_CODE_ACTION_ID_NOT_SUPPORT, "");
                                break;
                            }
                            case NAR_ACTION_PARAM_0_NOT_SUPPORT: {
                                SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_PARAM_0_NOT_SUPPORT, "");
                                throw EXEC_ERROR(ERROR_CODE_ACTION_PARAM_0_NOT_SUPPORT, "");
                                break;
                            }
                            case NAR_ACTION_PARAM_1_NOT_SUPPORT: {
                                SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_PARAM_1_NOT_SUPPORT, "");
                                throw EXEC_ERROR(ERROR_CODE_ACTION_PARAM_1_NOT_SUPPORT, "");
                                break;
                            }
                            default: {
                                SET_ACTION_EXEC_FAILED(ERROR_CODE_UNREACHABLE, "EAC get unknown error!");
                                throw EXEC_ERROR(ERROR_CODE_UNREACHABLE, "EAC get unknown error!");
                                break;
                            }
                        }
                    } else {
                        mm->result_state = RESPONSE_OK;
                    }

                    action_execute_time_ms_ = 0;
                    action_finished_time_ms_ = -1;
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
                        TaskManager::getInstance()->setActionFinishCanceled();
                        if (driver_.cancelAction()) {
                            mm->result_state = RESPONSE_OK;
                            break;
                        } else {
                            throw EXEC_ERROR(ERROR_CODE_ACTION_RESPONSE_TIMEOUT, "");
                        }
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

void ExtensionActionController::onActionCmdMsg(core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<core::CommandMsg>(msg);
    auto session_id = mm->session_id;

    try {
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
                        if (driver_.pauseAction()) {
                            mm->result_state = RESPONSE_OK;
                            break;
                        } else {
                            throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_PAUSE_FAILED, "");
                        }
                    }
                }
                return;
                break;
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
                        if (driver_.continueAction()) {
                            mm->result_state = RESPONSE_OK;
                            break;
                        } else {
                            throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_CONTINUE_FAILED, "");
                        }
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
        // 命令处理出错的情况
        mm->result_state = RESPONSE_FAILED;
        mm->result_code = e.errorCode();
        responseCommand(mm);
        return;
    }

    responseCommand(mm);
}

void ExtensionActionController::onEmergencyStateMsg(core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<core::EmergencyMsg>(msg);
    driver_.setEmergency(mm->emergency_state == STATE_EMERGENCY_TRIGER ||
                         mm->emergency_state == STATE_EMERGENCY_RECOVERABLE);

    if (mm->emergency_state == STATE_EMERGENCY_NONE) {
        driver_.reset();
        LOGGER(INFO, CMD_HANDER) << "send reset request to eac!";
    }

}

void ExtensionActionController::onTimer_100ms(core::base_msg_ptr msg) {
    if (!strict_model_) {
        action_execute_time_ms_ += 100;
        if (action_finished_time_ms_ != -1) {
            action_finished_time_ms_ += 100;
        }
    }

    if (!synchronizer_.connected()) {
        eac_device_->setStateTimeout();
        return;
    }

    int error_no;
    eac_device_system_state_ = driver_.getSystemState(error_no);
//    LOG(INFO) << "system state is " << eac_device_system_state_ << " error no is " << error_no;

    if (synchronizer_.synchronizer_ok()) {
        static int old_heat_beat = 0;
        auto heat_beat = 0;
        if (eac_disable_heat_beat_) {
            if (eac_device_system_state_ != SYSTEM_STATE_NONE) {  // 能读取到寄存器，系统状态不对，应该也任务心跳有问题
                heat_beat = old_heat_beat + 1;
            }
        } else {
            heat_beat = driver_.getHeartbeat();
        }
        if (old_heat_beat != heat_beat) {
            eac_device_->keepAlive();
            if (old_heat_beat == 0) {  // 第一次改变心跳时，就获取一次版本号，防止一直获取，一直获取可能占资源
                eac_device_->setVersionNo(driver_.getVersion());
                eac_device_->setModelNo(std::to_string(driver_.getType()));
                updateMusic(driver_.getType());
            }

            old_heat_beat = heat_beat;

            // 心跳改变后才改变硬件状态，否则可能没有心跳了，系统状态还显示正常，就会出现一会掉线一会在线
            switch (eac_device_system_state_) {
                case SYSTEM_STATE_NONE: {
                    break;
                }
                case SYSTEM_STATE_INIT: {
                    eac_device_->setStateInitialization();
                    break;
                }
                case SYSTEM_STATE_IDLE: {
                    eac_device_->setStateOK();
                    break;
                }
                case SYSTEM_STATE_RUNNING: {
                    eac_device_->setStateOK();
                    break;
                }
                case SYSTEM_STATE_ERROR: {
                    eac_device_->setStateError(error_no);
                    break;
                }
                default: {
                    LOG(WARNING) << "UNREACHABLE!";
                    break;
                }
            }
        }

        if (g_state.action_unit == sros::core::ACTION_UNIT_EAC) {
            auto multi_load_state = driver_.getMultiLoadState();
            if (g_state.multi_load_state != multi_load_state) {
                g_state.multi_load_state = multi_load_state;
                LOG(INFO) << "multi_load_state: " << g_state.multi_load_state;
            }
            sros::core::LoadState new_load_state =
                multi_load_state == 0 ? sros::core::LOAD_FREE : sros::core::LOAD_FULL;
            if (g_state.load_state != new_load_state) {
                g_state.load_state = new_load_state;

                if (new_load_state == sros::core::LOAD_FULL) {
                    LOG(INFO) << "Load state changed: LOAD_FREE -> LOAD_FULL";
                } else {
                    LOG(INFO) << "Load state changed: LOAD_FULL -> LOAD_FREE";
                }
            }
        }

        g_state.eac_prohibit_movement_task = driver_.isMovementFrohibition();

        updateActionTaskState();
        updateSrosTaskState();
    }
}

void ExtensionActionController::onTimer_1s(core::base_msg_ptr msg) {
    if (g_state.sys_state == SYS_STATE_IDLE && !synchronizer_.isRunning()) {
        synchronizer_.run();  // 待系统启动完成后，才让其启动，否则很有可能将一些不正确的数据传过去了
    }
}

void ExtensionActionController::updateActionTaskState() {
    auto action_task = core::TaskManager::getInstance()->getActionTask();
    if (action_task && action_task->isRunning() && !action_task->isWaitForStart() && cur_eac_task_) 
    {
        // 扩展动作id，用户开发的控制器，定制的动作
        int action_id = cur_eac_task_->getActionID();
        if (!(action_id >= 0xC0 && action_id <= 0xFF)) {
            return;
        }
        
        /**
         * 设备系统空闲，或是正在运行的时候采取查询，其他的时候没有必要，系统空闲的状态去查询的原因是，
         * 可能设备已经空闲了，但是src还处于运行状态，有一定的滞后性
         */
        if (eac_device_system_state_ == SYSTEM_STATE_IDLE || 
            eac_device_system_state_ == SYSTEM_STATE_RUNNING ||
            eac_device_system_state_ == SYSTEM_STATE_ERROR) {
            ActionStatus action_status;
            driver_.getActionStatus(action_status);
            if (action_status.action_no_ != action_task->getTaskNo()) {
                LOG(INFO) << "action no from sros and eac : " << action_task->getTaskNo() << "," << action_status.action_no_;
                action_status.action_no_ = action_task->getTaskNo();
            }
            
            switch (action_status.action_state_) {
                case TASK_WAIT_FOR_START:
                case TASK_RUNNING:
                case TASK_PAUSED:
                case TASK_IN_CANCEL: {
                    TaskManager::getInstance()->updateActionState(action_status.action_state_);
                    break;
                }
                case TASK_FINISHED: {
                    if (!strict_model_ && action_execute_time_ms_ < DELAY_DETECTION_TIME) {
                        LOGGER(ERROR, SROS) << "Finished immediately in " << action_execute_time_ms_ << "ms!";
                        break;
                    }
                    if (action_finished_time_ms_ == -1) {
                        action_finished_time_ms_ = 0;
                    }
                    auto task_manager = TaskManager::getInstance();
                    switch (action_status.action_result_) {
                        case TASK_RESULT_OK:
                        case TASK_RESULT_CANCELED:
                        case TASK_RESULT_FAILED: {
                            responseEacActionResult(action_status);
                            break;
                        }
                        default: {
                            if (!strict_model_ && action_finished_time_ms_ < DELAY_DETECTION_TIME) {
                                LOGGER(ERROR, SROS)
                                    << "Finished with wrong ActionResult:" << action_status.action_result_
                                    << " in " << action_finished_time_ms_ << "ms!";
                                break;
                            }
                            responseEacActionResult(action_status);
                            break;
                        }
                    }

                    int error_no;
                    eac_device_system_state_ = driver_.getSystemState(error_no);
                    break;
                }
                default: {
                    LOGGER(ERROR, SROS) << "Get wrong action state: " << action_status.action_state_
                                        << ", please check EAC ActionState.";
                    if (!strict_model_ && action_execute_time_ms_ < DELAY_DETECTION_TIME) {
                    } else {
                        if (eac_device_system_state_ != SYSTEM_STATE_RUNNING) {
                            action_status.action_result_ = TASK_RESULT_FAILED;
                            responseEacActionResult(action_status);
                        }
                    }
                    break;
                }
            }
        }
    }
}

void ExtensionActionController::updateSrosTaskState() {
    if (eac_device_system_state_ == SYSTEM_STATE_RUNNING) {
        SrosActionState status;
        driver_.getSrosActionStatus(status);
        static SrosActionControl old_sros_action_control = SROS_ACTION_CONTROL_NONE;
        if (old_sros_action_control != status.sros_action_control_) {
            auto action_task = std::atomic_load(&sros_sub_task_);
            switch (status.sros_action_control_) {
                case SROS_ACTION_CONTROL_NONE: {
                    if (action_task && action_task->isFinished()) {
                        // 没有启动EAC子任务，直接退出
                    } else {
                        // SROS子任务执行过程中，SrosActionControl设置为了0，强制结束，并且报错
                        // FIXME(pengjiali): SROS子任务
                        LOG(ERROR) << "SROS sub action still in running! can not be none!";
                        return;
                    }
                    break;
                }
                case SROS_ACTION_CONTROL_CLEAN: {
                    if (action_task && !action_task->isFinished()) {
                        LOG(ERROR) << "SROS sub action still in running! can not clean!";
                        return;
                    } else {
                        driver_.cleanSrosAction();
                    }

                    break;
                }
                case SROS_ACTION_CONTROL_START: {
                    if (action_task && !action_task->isFinished()) {
                        LOG(ERROR) << "SROS sub action still in running! can start again!";
                        return;
                    } else {
                        if (status.sros_action_id_ != 40) {
                            LOG(ERROR) << "SROS sub action id not support! action id 40 only support.";
                            return;
                        }

                        action_task = std::make_shared<sros::core::ActionTask>(
                            0, getName(), status.sros_action_id_, status.sros_action_param_0_,
                            status.sros_action_param_1_, status.sros_action_param_2_);
                        action_task->updateState(TASK_RUNNING);
                        std::atomic_store(&sros_sub_task_, action_task);
                        driver_.updateSrosActionStatus(TASK_RUNNING, TASK_RESULT_NA, 0);
                        // 绑定SRCActionState回调
                        src_sdk->setActionStateCallback(
                            boost::bind(&ExtensionActionController::onSRCActionState, this, _1));
                        src_sdk->executeActionInt(action_task->getTaskNo(), status.sros_action_id_,
                                               (uint16_t)(int16_t)status.sros_action_param_0_, (uint16_t)(int16_t)status.sros_action_param_1_,
                                               (int32_t)status.sros_action_param_2_);
                        LOG(INFO) << "######## SRC::executeAction(): "
                                  << "no " << action_task->getTaskNo() << ", "
                                  << "id " << status.sros_action_id_ << ", "
                                  << "p0 " << status.sros_action_param_0_ << ", "
                                  << "p1 " << status.sros_action_param_1_ << ", "
                                  << "p2 " << status.sros_action_param_2_;
                    }
                    break;
                }
                case SROS_ACTION_CONTROL_PAUSE: {
                    LOG(ERROR) << "The pause function has not yet implemented!";
                    break;
                }
                case SROS_ACTION_CONTROL_CONTINUE: {
                    LOG(ERROR) << "The continue function has not yet implemented!";
                    break;
                }
                case SROS_ACTION_CONTROL_CANCEL: {
                    if (!action_task || !action_task->isRunning()) {
                        LOG(ERROR) << "SROS sub action not running! can not cancel!";
                        return;
                    } else {
                        src_sdk->cancelAction(action_task->getTaskNo());
                        action_task->finishTask(TASK_RESULT_CANCELED, 0);
                        std::atomic_store(&sros_sub_task_, action_task);
                        driver_.updateSrosActionStatus(TASK_FINISHED, TASK_RESULT_CANCELED, 0);
                    }
                    break;
                }
                default: {
                    LOG(ERROR) << "unreachable! sros action control is " << status.sros_action_control_;
                    break;
                }
            }
            old_sros_action_control = status.sros_action_control_;
        }
    }
}

void ExtensionActionController::responseCommand(const core::CommandMsg_ptr &msg) {
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

void ExtensionActionController::responseEacActionResult(int action_no, TaskResult act_task_state, int result_code) 
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
void ExtensionActionController::responseEacActionResult(const sros::modbus::ActionStatus& _status)
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

void ExtensionActionController::updateMusic(int eac_type) {
    if (195 <= eac_type && eac_type <= 199) {  // 遨博机械臂，这时候报语音机械臂故障
        auto fault_center = FaultCenter::getInstance();
        fault_center->setMusicId(core::FAULT_CODE_EAC_UNKNOWN_FAULT, hmi::MUSIC_ID_MUSIC_ROBOTIC_ARM_ERROR);
        fault_center->setMusicId(core::FAULT_CODE_EAC_CONNECT_FAILED, hmi::MUSIC_ID_MUSIC_ROBOTIC_ARM_ERROR);
        fault_center->setMusicId(core::FAULT_CODE_EAC_INITIAL_FAILED, hmi::MUSIC_ID_MUSIC_ROBOTIC_ARM_ERROR);
        fault_center->setMusicId(core::FAULT_CODE_EAC_TIMEOUT, hmi::MUSIC_ID_MUSIC_ROBOTIC_ARM_TIMEOUT);
        fault_center->setMusicId(core::FAULT_CODE_EAC_OTHER_FAULT, hmi::MUSIC_ID_MUSIC_ROBOTIC_ARM_ERROR);
    }
}

void ExtensionActionController::onSRCActionState(sros::core::TaskStateStruct src_action_state) {
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
                    driver_.updateSrosActionStatus(TASK_FINISHED, TASK_RESULT_OK, action_result_value);
                    break;
                }
                case TASK_RESULT_CANCELED: {
                    LOG(INFO) << "SRC_ACTION_RESULT_CANCELED";
                    action_task->finishTask(TASK_RESULT_CANCELED, action_result_value);
                    driver_.updateSrosActionStatus(TASK_FINISHED, TASK_RESULT_CANCELED, action_result_value);
                    break;
                }
                case TASK_RESULT_FAILED: {
                    LOG(INFO) << "SRC_ACTION_RESULT_FAILED";
                    action_task->finishTask(TASK_RESULT_FAILED, action_result_value);
                    driver_.updateSrosActionStatus(TASK_FINISHED, TASK_RESULT_FAILED, action_result_value);
                    break;
                }
                default: {
                    // 接收到了非法的result值
                    LOG(ERROR) << "Wrong src action result " << action_result;
                    action_task->finishTask(TASK_RESULT_FAILED, action_result_value);
                    driver_.updateSrosActionStatus(TASK_FINISHED, TASK_RESULT_FAILED, action_result_value);
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