//
// Created by caoyan on 1/9/21.
//

#include "action_controller_module.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <memory>

#include "core/settings.h"

#include "core/exec_error.hpp"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/msg/str_msg.hpp"
#include "core/msg/usart_data_msg.hpp"
#include "core/src.h"

#include "rfid_manager.h"
#include "action_manager.h"

using namespace std;
using namespace sros::core;

namespace ac {

ActionControllerModule::ActionControllerModule() : Module("ActionController") {}

ActionControllerModule::~ActionControllerModule() {}



void ActionControllerModule::run() {
    //    waitForStartCommand();

    LOG(INFO) << "ActionController module start running";

    subscribeTopic("DEBUG_CMD", CALLBACK(&ActionControllerModule::onDebugCmdMsg));          //调试指令
    subscribeTopic("ACTION_CMD", CALLBACK(&ActionControllerModule::onActionCmdMsg));        //动作指令
    subscribeTopic("TOPIC_NOTIFY", CALLBACK(&ActionControllerModule::onNotifyMsg));         //移动返回通知
    subscribeTopic("DETECT_RESULT", CALLBACK(&ActionControllerModule::onAlgoDetectResult)); //算法返回结果
    subscribeTopic("TOPIC_POSTURE_CORRECT_RESULT", CALLBACK(&ActionControllerModule::onAlgoDetectResult)); //位姿矫正算法结果返回
    subscribeTopic("EAC_ACTION_RESULT", CALLBACK(&ActionControllerModule::onEacActionResult));//eac动作结果返回
    subscribeTopic("LASER_REGION_STATE", CALLBACK(&ActionControllerModule::onLaserRegionObstacleMsg)); //避障激光数据
    subscribeTopic("NAV_PATH_BUILD_RESULT", CALLBACK(&ActionControllerModule::onNavBuildResult)); //NAV生成的路径结果
    subscribeTopic("TIMER_50MS", CALLBACK(&ActionControllerModule::onTimer_50ms));

    auto &s = sros::core::Settings::getInstance();
    if (s.getValue<std::string>("device.enable_rfid", "False") == "True") {
        auto rfid_device_name = s.getValue<std::string>("device.rfid_serial_device", "/dev/ttyUSB0");
        auto rfid_baud_rate = s.getValue<unsigned int>("device.rfid_serial_baud_rate", 57600);
        if(!RfidManager::getInstance()->init(rfid_device_name, rfid_baud_rate)) {
            LOG(ERROR) << "start rfid fail";
        } else {
            LOG(INFO) << "start rfid success";
        }
        
    } else {
        LOG(INFO) << "未启动rfid";
    }

    // 绑定SRCActionState回调
    src_sdk->setActionStateCallback(boost::bind(&ActionControllerModule::onSRCActionState, this, _1));

    ActionManager::getInstance()->init();

    dispatch();
}

void ActionControllerModule::onEacActionResult(const sros::core::base_msg_ptr msg) {
    if(msg == nullptr) {
        return;
    }

    auto eac_act_result_msg = dynamic_pointer_cast<sros::core::CommonStateMsg<TaskResult>>(msg);
    auto eac_act_result_state = eac_act_result_msg->state;
    auto eac_act_no = eac_act_result_msg->param_int;
    auto eac_act_result_code = eac_act_result_msg->failed_code_;
    auto eac_str_reserved = eac_act_result_msg->str_reserved;

    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (!action_task || action_task->getTaskNo() != eac_act_no || !action_task->isSlaveRunning()) {
        return;
    }

    if (eac_act_result_code == 0 || (int)eac_act_result_state == 0){
        LOGGER(ERROR, ACTION_TASK) << "get Task State From EAC is Error!!!, please Eac develop Check there program!!!!!!!! ";
    }

    LOG(INFO) << "=====> TASK: no " << eac_act_no << " finished with value = " << (int)eac_act_result_state
              << " action_result = " << eac_act_result_code;

    ActionManager::getInstance()->onAcFinish(action_task);

    switch (eac_act_result_state) {
        case TASK_RESULT_OK: {

            //下发匹配串进行比较,同时兼容
            std::string action_param_str = action_task->getActionParamStr();
            std::string qr_code = std::to_string(eac_act_result_code);

            if(!action_param_str.empty()) {
                if(qr_code != action_param_str) {
                    LOG(ERROR) << "match fail, eac sacn info(" << qr_code << ") != param_str(" << action_param_str << ")";
                    action_task->setResultValueStr(qr_code);
                    ActionManager::getInstance()->onAcFinishFailed(action_task, sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                    return;
                } else {
                    action_task->setResultValueStr(qr_code);
                }
            } else {
                action_task->setResultValueStr(eac_str_reserved);
            }

            ActionManager::getInstance()->onAcFinishSucceed(action_task, eac_act_result_code);
            break;
        }
        case TASK_RESULT_CANCELED: {
            ActionManager::getInstance()->onAcFinishCanceled(action_task, eac_act_result_code);
            break;
        }
        case TASK_RESULT_FAILED: {
            ActionManager::getInstance()->onAcFinishFailed(action_task, eac_act_result_code);
            break;
        }
    }

}

void ActionControllerModule::onAlgoDetectResult(const sros::core::base_msg_ptr msg) {
    if(msg == nullptr) {
        return;
    }

    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (!action_task || !action_task->isSlaveRunning()) {
        return;
    }

    ActionManager::getInstance()->onAlgoCallback(action_task, msg);
}

void ActionControllerModule::onLaserRegionObstacleMsg(const sros::core::base_msg_ptr msg) {
    if(msg == nullptr) {
        return;
    }

    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (!action_task || !action_task->isSlaveRunning()) {
        return;
    }

    ActionManager::getInstance()->onRegionObstacleMsg(action_task, msg);
}

void ActionControllerModule::onNavBuildResult(const sros::core::base_msg_ptr msg) {
    if(msg == nullptr) {
        return;
    }

    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (!action_task || !action_task->isSlaveRunning()) {
        return;
    }

    ActionManager::getInstance()->onNavCallback(action_task, msg);
}

void ActionControllerModule::onNotifyMsg(sros::core::base_msg_ptr m) {
    if(m == nullptr) {
        return;
    }

    auto notify_msg = dynamic_pointer_cast<sros::core::NotificationMsg>(m);

    if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED) {
        auto move_task = notify_msg->movement_task;

        if (move_task->getTaskSourceModule() != getName()) {
            return;
        }

        auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
        if (!action_task || !action_task->isSlaveRunning()) {
            return;
        }

        TaskResult movement_task_result_ = move_task->getTaskResult();
        bool ret = (movement_task_result_ == TaskResult::TASK_RESULT_OK);
        if (ret) {
            LOG(INFO) << "Move task execute successfully";
            ActionManager::getInstance()->onMcFinishSucceed(action_task, 0);

        } else {
            LOG(ERROR) << "Move task execute failed, result: " << move_task->getTaskResult()
                                << " failed code: " << move_task->getFailedCode();
            ActionManager::getInstance()->onMcFinishFailed(action_task, move_task->getFailedCode());
        }
    }
}

/**
 * 接收NetworkTask将其发送给动作控制器
 * @param msg
 */
void ActionControllerModule::onDebugCmdMsg(sros::core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    auto session_id = mm->session_id;
    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            throw EXEC_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", g_state.control_mutex.getLockerSessionId(),
                             "current:", session_id);
        }
    };

    try {
        switch (mm->command) {
            case sros::core::CMD_INPUT_ACTION_VALUE: {
                LOGGER(INFO, CMD_HANDER) << "Handing CMD_INPUT_ACTION_VALUE command, action value is " << mm->param0;

                getControlMutexFun();

                auto action_task = sros::core::TaskManager::getInstance()->getActionTask();

                if (!action_task || action_task->getActionID() != ACTION_ID_WAIT_COMMAND) {
                    throw EXEC_ERROR(ERROR_CODE_UNDEFINED, "action id is not", ACTION_ID_WAIT_COMMAND);
                }

                // 若放行已经结束了，处于等ack的状态时，防止其再次确认进入finish状态
                if (!action_task->isSlaveRunning()) {
                    mm->result_state = RESPONSE_OK;
                } else if (action_task->getActionParam() == mm->param0) {
                    TaskManager::getInstance()->setActionFinishSucceed();

                    mm->result_state = RESPONSE_OK;
                }
                break;
            }
            case sros::core::CMD_NEW_ACTION_TASK:
            case sros::core::CMD_CANCEL_ACTION_TASK: {
                // in onActionCmdMsg process
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

    responseCommand(mm);
}

void ActionControllerModule::onActionCmdMsg(sros::core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    auto session_id = mm->session_id;

    try {
        switch (mm->command) {
            case sros::core::CMD_NEW_ACTION_TASK: {
                auto action_task = mm->action_task;
                if (!action_task) {
                    LOGGER(WARNING, CMD_HANDER) << "action task is NULL";

                    return;
                }

                auto action_id = action_task->getActionID();
                if (action_id == 0x4f || action_id == 0x4e) {
                    return;
                }

                bool bResult = false;
                // EAC Action
                if ((action_id >= 0xC0 && action_id <= 0xFF)) {
                    auto &s = sros::core::Settings::getInstance();
                    if (s.getValue<std::string>("device.enable_eac", "False") == "False") {
                        SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_DISABLED, "EAC disabled");
                        throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_DISABLED, "EAC disabled");
                    } else {
                        sros::core::TaskManager::getInstance()->updateActionTask(action_task);
                        bResult = ActionManager::getInstance()->onStartEacAction(action_task);
                    }
                }
                // SROS Action
                else if ((action_id >= 0x80 && action_id < 0xBF)) {
                    sros::core::TaskManager::getInstance()->updateActionTask(action_task);
                    src_sdk->setActionStateCallback(boost::bind(&ActionControllerModule::onSRCActionState, this, _1));
                    bResult = ActionManager::getInstance()->onStartSrosAction(action_task);
                }
                // SRC Action
                else if ((action_id > 0 && action_id < 0x7F)) {
                    sros::core::TaskManager::getInstance()->updateActionTask(action_task);
                    src_sdk->setActionStateCallback(boost::bind(&ActionControllerModule::onSRCActionState, this, _1));
                    bResult = ActionManager::getInstance()->onStartSrcAction(action_task);
                }

                if (!bResult) {
                    SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_ID_NOT_SUPPORT, "action_id not support! action_id is",
                                           action_id);
                    throw EXEC_ERROR(ERROR_CODE_ACTION_ID_NOT_SUPPORT, "action_id not support! action_id is",
                                     action_id);
                }

                mm->result_state = RESPONSE_OK;
                break;
            }
            case sros::core::CMD_CANCEL_ACTION_TASK: {
                auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
                if (!action_task) {
                    LOGGER(WARNING, CMD_HANDER) << "action task is NULL";
                }

                if (action_task->isSlaveRunning() || action_task->isWaitForStart()) {
                    int action_id = action_task->getActionID();

                    //VSC return
                    if(action_id == 0x4e || action_id == 0x4f) {
                        //do nothing
                        return;
                    }
     
                    int reason = 0;
                    if(mm->user_name == "manual_btn") {
                        reason = sros::core::ERROR_CODE_MANUAL_CONTROL_MOVEMENT_RUNNING;
                    }

                    if((action_id >= 0xC0 && action_id <= 0xFF)) { //EAC
                        // in extension action controller handle
                        auto &s = sros::core::Settings::getInstance();
                        if (s.getValue<std::string>("device.enable_eac", "False") == "False") {
                            SET_ACTION_EXEC_FAILED(ERROR_CODE_ACTION_EAC_DISABLED, "EAC disabled");
                            throw EXEC_ERROR(ERROR_CODE_ACTION_EAC_DISABLED, "EAC disabled");
                        }

                        ActionManager::getInstance()->onCancelEacAction(action_task, reason);

                    } else if ((action_id >= 0x80 && action_id < 0xBF)) {   //SROS
                        ActionManager::getInstance()->onCancelSroslAction(action_task, reason);
                    } else if ((action_id > 0 && action_id < 0x7F)) {       //SRC
                        ActionManager::getInstance()->onCancelSrcAction(action_task, reason);
                    }
                    mm->result_state = RESPONSE_OK;
                }
                break;
            }
            default: {
                return;  // 不是由此处处理，也不会消息
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

void ActionControllerModule::responseCommand(const sros::core::CommandMsg_ptr &msg) {
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

void ActionControllerModule::onSRCActionState(sros::core::TaskStateStruct src_action_state) {
    // 在这里更新ActionTask的状态
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (!action_task) return;

    const auto &action_no = src_action_state.no_;
    uint32_t actionTaskNo;
    if (!ActionManager::getInstance()->onSubSrcActionCheck(action_task, actionTaskNo)) {
        actionTaskNo = action_task->getTaskNo();
    }

    if (actionTaskNo != action_no || !action_task->isSlaveRunning()) {
        return;
    }

    switch (src_action_state.state_) {
        case TASK_RUNNING: {
            LOG(INFO) << "action " << action_no << " is running ";
            TaskManager::getInstance()->setActionRunning();
            break;
        }
        case TASK_WAIT_FOR_START:
        case TASK_PAUSED:
        case TASK_IN_CANCEL: {
            TaskManager::getInstance()->updateActionState(src_action_state.state_);
            break;
        }
        case TASK_FINISHED: {
            LOGGER(INFO, ACTION_TASK) << "=====> TASK: no " << action_no << " finished with value = " << src_action_state.result_
                      << " action_result = " << src_action_state.result_value_;

            if(!ActionManager::getInstance()->onAcFinish(action_task)) {
                break;
            }

            switch (src_action_state.result_) {
                case TASK_RESULT_NA: {
                    SET_ACTION_EXEC_FAILED(src_action_state.result_value_,
                                           "wrong action result:", src_action_state.result_value_);
                    break;
                }
                case TASK_RESULT_OK: {
                    LOG(INFO) << "SRC_ACTION_RESULT_OK";

                    ActionManager::getInstance()->onAcFinishSucceed(action_task, src_action_state.result_value_);
                    break;
                }
                case TASK_RESULT_CANCELED: {
                    LOG(INFO) << "SRC_ACTION_RESULT_CANCELED";
                    ActionManager::getInstance()->onAcFinishCanceled(action_task, src_action_state.result_value_);
                    break;
                }
                case TASK_RESULT_FAILED: {
                    LOG(INFO) << "SRC_ACTION_RESULT_FAILED";
                    ActionManager::getInstance()->onAcFinishFailed(action_task, src_action_state.result_value_);
                    break;
                }
            }
            break;
        }
        default: {
            SET_ACTION_EXEC_FAILED(src_action_state.state_, "wrong action state:", src_action_state.state_);
            break;
        }
    }
}

void ActionControllerModule::onTimer_50ms(sros::core::base_msg_ptr msg) {
    auto cur_time = sros::core::util::get_time_in_ms();
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();

    ActionManager::getInstance()->onTimer50ms(action_task, cur_time);

}

} /* namespace ac */
