#include "mission_module.h"

#include <glog/logging.h>
#include "core/exec_error.hpp"
#include "core/msg/modbus_register_msg.hpp"
#include "core/state.h"
#include "core/task/task_manager.h"

using namespace std;

namespace sros {

MissionModule::MissionModule(const std::string &module_name)
    : Module(module_name),
      manual_stop_(false),
      parallel_action_has_started_(false),
      last_sys_state_(SystemState::SYS_STATE_ZERO),
      movement_task_result_(TaskResult::TASK_RESULT_NA),
      action_task_result_(TaskResult::TASK_RESULT_NA),
      reg_admin_(RegisterAdmin::getInstance()),
      mission_manager_(MissionManager::getInstance()) {}

MissionModule::~MissionModule() {}

void MissionModule::run() {
    LOG(INFO) << "MissionModule: run";

    subscribeTopic("DEBUG_CMD", CALLBACK(&MissionModule::onDebugCmdMsg));
    subscribeTopic("TOPIC_CMD_RESPONSE", CALLBACK(&MissionModule::onCommandResponseMsg));
    subscribeTopic("TOPIC_NOTIFY", CALLBACK(&MissionModule::onNotifyMsg));  // 接收动作执行结果
    subscribeTopic("TIMER_200MS", CALLBACK(&MissionModule::onTimer200ms));
    dispatch();
}

void MissionModule::execNextMission() {
    if (mission_manager_->pendingMissionSize() <= 0) {
        return;
    }

    if (manual_stop_) {
        manual_stop_ = false;
    }

    mission_manager_->clearParentMissions();
    auto mission = mission_manager_->dequeueMission();

    // 从数据库中恢复任务时,需要根据当前步骤依次恢复出子任务层次
    vector<AbstractStepPtr> child_mission_steps;
    if (!mission->traverseStep(child_mission_steps)) {
        // 未找到子任务
        string err_step_id = getCurStepId();
        mission_manager_->setToRootMission();
        getCurrentMission()->result_ = MissionResult::MISSION_RESULT_FAILED;
        getCurrentMission()->cur_step_id_ = err_step_id;
        getCurrentMission()->err_code_ = 0;
        stopMission();
        return;
    }

    if (child_mission_steps.size() > 0) {
        // 根任务状态保存到数据库
        storage_.updateMissionState(mission->no_, MissionState::RUNNING);

        // 当前任务id实际上是最终子任务child_mission_step[length-1]的当前任务id
        // 各任务的当前任务id由任务之间的层次关系确定
        string cur_step_id = mission->currentStepId();
        for (uint32_t idx = 0; idx < child_mission_steps.size(); idx++) {
            auto step = std::dynamic_pointer_cast<StepMission>(child_mission_steps.at(idx));
            if (!step) {
                LOG(ERROR) << "Null mission step";
                return;
            }

            mission->setCurStepId(step->getStepId());
            mission->state_ = MissionState::RUNNING;
            mission = prepareChildMission(step, "");
            if (idx == child_mission_steps.size() - 1) {
                mission->setCurStepId(cur_step_id);
            }
        }
    }

    startMission(mission);
    //    notifyMissionListChanged();
}

bool MissionModule::startMission(MissionInstancePtr mission) {
    if (!mission) {
        return false;
    }

    if (getCurStepId() == "") {
        setCurStepId(getStartStepId());
    }

    LOGGER(INFO, TASK) << "Start task " << mission->mission_info_.name_ << " from step " <<  getCurStepId();

//    storage_.recordMissionStart(mission->no_, mission->getId(), mission->user_name_);
    getCurrentMission()->state_ = MissionState::RUNNING;

    // 启动的子任务并不会存储到数据库,也不会显示到任务队列
    if (!getCurrentMission()->isChildMission) {
        storage_.updateMissionState(mission->no_, MissionState::RUNNING);
    }

    AbstractStepPtr mission_step = getExecMissionStep();
    if (!doExecuteStep(mission_step)) {
        return false;
        //        execNextMission();
    }
    return true;
}

bool MissionModule::cancelMission(uint64_t mission_no) {
    // 如果任务未开始，则从任务队列中删除该任务
    if (mission_manager_->hasPendingMission(mission_no)) {
        auto item = mission_manager_->removeMission(mission_no);
        if (!item) {
            return false;
        }

        item->state_ = MissionState::FINISHED;
        item->result_ = MissionResult::MISSION_RESULT_CANCELED;
        item->cur_step_id_ = "";
        item->finish_timestamp_ = sros::core::util::get_timestamp_in_ms();

        storage_.updateMissionState(mission_no, item->state_);
        storage_.setMissionResult(mission_no, item->result_, item->err_code_);

        //        addFinishedMission(item);
        notifyMissionListChanged();

        LOGGER(INFO, TASK) << "Remove task " << item->getName() << " from task queue";
        return false;
    }

    // 由于子任务启动时使用父任务的mission_no作为子任务的mission_no
    // 因此不会有问题
    if (isRunning() && getCurMissionNo() == mission_no) {
        string err_step_id = getCurStepId();
        mission_manager_->setToRootMission(); // 终止顶层mission

        auto cur_task = getCurrentMission();
        cur_task->cur_step_id_ = err_step_id;  // 需要先回到最顶层mission后设置

        LOGGER(INFO, TASK) << "Stop running task " << cur_task->getName() << ", task id: " << cur_task->no_;

        manual_stop_ = true;
        stopMission(manual_stop_);
        //        setCurStepId("");
        return true;
    }

    return false;
}

bool MissionModule::stopMission(bool manual_stop) {
    MissionInstancePtr cur_mission_ptr = getCurrentMission();
    if (manual_stop) {
        cur_mission_ptr->result_ = MissionResult::MISSION_RESULT_CANCELED;
    }
    cur_mission_ptr->state_ = MissionState::FINISHED;
    cur_mission_ptr->finish_timestamp_ = sros::core::util::get_timestamp_in_ms();
    //    setCurStepId("");

    LOGGER(INFO, TASK) << "Stop task " + cur_mission_ptr->getName() << " task id: " << cur_mission_ptr->getNo();

    mission_manager_->addFinishedMission(getCurrentMission());

    storage_.updateMissionState(cur_mission_ptr->no_, cur_mission_ptr->state_);
    storage_.updateMissionCurrentStep(cur_mission_ptr->no_, cur_mission_ptr->cur_step_id_);
    storage_.setMissionResult(cur_mission_ptr->no_, cur_mission_ptr->result_, cur_mission_ptr->err_code_);

    notifyMissionListChanged();
    return true;
}

void MissionModule::onDebugCmdMsg(core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    auto session_id = msg->session_id;
    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            auto locker = g_state.control_mutex.getLocker();
            SET_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", locker.session_id, "current:", session_id);
            auto msg = dynamic_pointer_cast<CommandMsg>(m);
            msg->result_state = RESPONSE_FAILED;
            msg->result_code = ERROR_CODE_CONTROL_MUTEX_IS_LOCKED;
            responseCommand(msg);

            return false;
        }
        return true;
    };

    switch (msg->command) {
        case CMD_START_MISSION: {
            if (!getControlMutexFun()) {
                break;
            }

            handleNewMission(msg);
            // 在上面的函数内部回复
            break;
        }
        case CMD_CANCEL_MISSION: {
            if (!getControlMutexFun()) {
                break;
            }

            handleCancelMission(msg);
            break;
        }
        case CMD_SRC_PAUSE: {
            if (!getControlMutexFun()) {
                break;
            }

            // 只是更新mission状态,main module模块会处理暂停功能
            handlePauseMovement(m);
            break;
        }
        case CMD_SRC_CONTINUE: {
            if (!getControlMutexFun()) {
                break;
            }

            // 只是更新mission状态,main module模块会处理继续功能
            handleContinueMovement(m);
            break;
        }
        case CMD_CONTINUE_MISSION: {
            if (!getControlMutexFun()) {
                break;
            }

            // 任务取消后，点击继续，执行下一个mission
            handleContinueMission(m);
            break;
        }
        case CMD_REORDER_MISSION: {
            if (!getControlMutexFun()) {
                break;
            }

            handleReorderMission(msg);
            break;
        }
        default: {
            break;
        }
    }
}

void MissionModule::onCommandResponseMsg(sros::core::base_msg_ptr m) {
    auto cmd_response = dynamic_pointer_cast<sros::core::CommandMsg>(m);
    if (cmd_response->source != getName()) {  // 这条命令不是由改模块发出的，所以拒收
        return;
    }

    core::ResultState result_state_ = cmd_response->result_state;
    uint32_t result_code_ = cmd_response->result_code;

    if(result_state_ == core::RESPONSE_FAILED) {

        LOG(INFO) << "result_state_: " << result_state_;
        LOG(INFO) << "result_code_: " << result_code_;

        if (isRunning()) {
            string err_step_id = getCurStepId();
            mission_manager_->setToRootMission(); // 终止顶层mission

            auto cur_task = getCurrentMission();
            cur_task->cur_step_id_ = err_step_id;  // 需要先回到最顶层mission后设置

            LOGGER(INFO, TASK) << "Stop running task " << cur_task->getName() << ", task id: " << cur_task->no_;

            manual_stop_ = true;
            stopMission(manual_stop_);
        }
    }

}

void MissionModule::onTimer200ms(sros::core::base_msg_ptr m) {
    auto last_state = last_sys_state_;
    last_sys_state_ = g_state.sys_state;

    // 定时更新电量值，否则如果mission需要判断电量，可能从寄存器获取到的值不是最新值
    // 即使任务没有启动，也需要更新电量值，否则会出现启动任务马上判断电量值，结果电量值
    // 与真实值不符
    reg_admin_->setInputRegister16(IRA_BATTERY_PERCENTAGE, g_state.battery_percentage);

    if (!isRunning()) {
        return;
    }

    AbstractStepPtr mission_step = getCurrentMission()->getMissionStep(getCurStepId());
    if (!mission_step) {
        return;
    }

    switch (mission_step->getStepType()) {
        // 等待寄存器更新mission step
        case MissionStepType::StepWaitRegUpdate: {
            StepWaitRegUpdatePtr wait_reg_update_ptr = std::dynamic_pointer_cast<StepWaitRegUpdate>(mission_step);
            if (!wait_reg_update_ptr) {
                return;
            }

            ModbusAddrType register_type = wait_reg_update_ptr->getRegisterType();
            uint16_t absolute_addr = reg_admin_->absoluteAddr(wait_reg_update_ptr->getRegisterAddr(), register_type);
            uint16_t data = wait_reg_update_ptr->getRegisterData();
            ExpressionType expression = wait_reg_update_ptr->getExpression();
            uint32_t cur_reg_dat = 0;

            // ===================================//
            // 兼容Matrix1.7.0及之前版本
            if (register_type == ModbusAddrType::AddrNone) {
                register_type = reg_admin_->getRegisterAddrType(absolute_addr);
            }
            // ===================================//

            if (!reg_admin_->doReadRegister(absolute_addr, cur_reg_dat, register_type)) {
                return;
            }

            if ((expression == ExpressionType::ExpEqual && cur_reg_dat == data) ||
                (expression == ExpressionType::ExpLessThan && cur_reg_dat < data) ||
                (expression == ExpressionType::ExpLarger && cur_reg_dat > data)) {
                LOGGER(INFO, TASK) << "Register has updated, addr: " << absolute_addr
                                         << " data: " << cur_reg_dat;
                if (!executeNextStep()) {
                    //                    execNextMission();
                }
            }
            break;
        }
        case MissionStepType ::StepWait: {
            StepWaitTimePtr wait_time_ptr = std::dynamic_pointer_cast<StepWaitTime>(mission_step);
            if (!wait_time_ptr) {
                return;
            }
            // 在定时过程中，如果用户点击了暂停，则不再计时，否则剩余时间减1（一个200ms时间单元）
            if (getCurrentMission()->state_ != MissionState::PAUSED) {
                wait_time_ptr->setRemainTime(wait_time_ptr->getRemainTime() - 1);

                // 定时时间到，执行下一步骤
                if (wait_time_ptr->getRemainTime() <= 0) {
                    // 重置剩余时间，否则影响循环任务的定时
                    wait_time_ptr->resetRemainTime();
                    // 任务被取消，定时时间到后不要执行
                    if (getCurrentMission()->state_ == MissionState::FINISHED) {
                        return;
                    }
                    LOGGER(INFO, TASK) << "Wait time out, execute next task step";

                    if (!executeNextStep()) {
                        //                        execNextMission();
                    }
                }
            }
            break;
        }
        case MissionStepType::StepParallel: {
            // 并行任务中，移动任务执行后同时执行动作任务
            if (!parallel_action_has_started_ && (g_state.sys_state == SystemState::SYS_STATE_TASK_NAV_WAITING_FINISH ||
            g_state.sys_state == SystemState::SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW)) {
                parallel_action_has_started_ = true; // 并行任务动作任务已经启动，避免多次执行
                StepParallelPtr parallelStep = std::dynamic_pointer_cast<StepParallel>(mission_step);
                if (!parallelStep) {
                    return;
                }
                auto step_action = parallelStep->getStep(MissionStepType::StepAction);
                if (step_action) {
                    doExecuteStepAction(step_action);
                }
            }
        }
        default: {
            break;
        }
    }
}

void MissionModule::onNotifyMsg(sros::core::base_msg_ptr m) {
    // 如果不是执行mission自己的任务，如移动到站点
    if (!isRunning()) {
        return;
    }

    AbstractStepPtr mission_step = getCurrentMission()->getMissionStep(getCurStepId());
    if (!mission_step) {
        return;
    }

    auto notify_msg = dynamic_pointer_cast<sros::core::NotificationMsg>(m);
    auto notify_type = notify_msg->notify_type;
    LOG(INFO) << "MissionModule::onNotifyMsg(): type=" + std::to_string(notify_type);
    switch (notify_type) {
        case sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED:
        case sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED: {
            bool ret = true;
            uint32_t err_code = 0;
            if (notify_type == sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED) {
                auto action_task = notify_msg->action_task;
                action_task_result_ = action_task->getTaskResult();
                ret = action_task_result_ == TASK_RESULT_OK;
                err_code = action_task->getActionResultValue();
                if (ret) {
                    LOGGER(INFO, TASK) << "Action task execute successfully";
                } else {
                    LOGGER(ERROR, TASK) << "Action task execute failed, result " << action_task->getTaskResult()
                                          << " failed code: " << action_task->getActionResultValue();
                }
            } else if (notify_type == sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED) {
                auto move_task = notify_msg->movement_task;

                if (move_task->getTaskSourceModule() != getName()) {
                   return;
                }

                movement_task_result_ = move_task->getTaskResult();
                ret = movement_task_result_ == TaskResult::TASK_RESULT_OK;
                err_code = move_task->getFailedCode();
                if (ret) {
                    LOGGER(INFO, TASK) << "Move task execute successfully";
                } else {
                    LOGGER(ERROR, TASK) << "Move task execute failed, result: " << move_task->getTaskResult()
                              << " failed code: " << move_task->getFailedCode() << " station: " << move_task->getDstStationNo();
                }
            }

            // 如果是并行任务，需要等所有步骤执行结束再继续下一个步骤
            if (mission_step->getStepType() == MissionStepType::StepParallel) {
                // 所有任务还没执行结束
                if (movement_task_result_ == TaskResult::TASK_RESULT_NA ||
                    action_task_result_ == TaskResult::TASK_RESULT_NA) {
                    return;
                } else {
                    // 只要一个步骤失败，则认为失败
                    ret = (movement_task_result_ == TaskResult::TASK_RESULT_OK &&
                            action_task_result_ == TaskResult::TASK_RESULT_OK);
                }
            }

            // 手动结束任务时，也会接收到移动或动作执行结束通知，
            // 此时并不需要更新result_等信息
            if (!ret && !manual_stop_) {
                // 执行移动任务或动作任务失败：
                // 如果下一个步骤是判断执行结果条件分支，则执行下一步骤
                // 否则，失败后直接结束任务
                AbstractStepPtr next_step = mission_manager_->getMissionNextStepInfo();
                if (next_step && next_step->getStepType() == MissionStepType::StepDecision) {
                    StepDecisionPtr decision_step = std::dynamic_pointer_cast<StepDecision>(next_step);
                    if (decision_step && decision_step->getDecisionType() == DecisionType::DecisionLastStepResult) {
                        executeNextStep(ret);
                        return;
                    }
                }

                // 可能存在mission嵌套,获取最开始启动的mission
                string err_step_id = getCurStepId();
                mission_manager_->setToRootMission();
                getCurrentMission()->result_ = MissionResult::MISSION_RESULT_FAILED;
                getCurrentMission()->cur_step_id_ = err_step_id;
                getCurrentMission()->err_code_ = err_code;
                stopMission();
                return;
            } else if (isRunning()) {
                // 执行下一个步骤
                if (!executeNextStep()) {
                    //                    execNextMission();
                }
            }
            break;
        }
        default: {
            break;
        }
    }
}

void MissionModule::responseCommand(const sros::core::CommandMsg_ptr &msg) {
    auto response_msg = std::make_shared<CommandMsg>(msg->source, "TOPIC_CMD_RESPONSE");
    response_msg->command = msg->command;
    response_msg->session_id = msg->session_id;
    response_msg->req_seq = msg->req_seq;
    response_msg->result_state = msg->result_state;
    response_msg->result_code = msg->result_code;
    sendMsg(response_msg);
}

void MissionModule::enqueueMission(uint64_t session_id, uint64_t no, uint32_t seq, uint32_t mission_id,
                                  std::string step_id, std::string user_name, ObstacleAvoidPolicy policy) {
    MissionInfo mission_info;
    if (!storage_.getMission(mission_id, mission_info)) {
        throw EXEC_ERROR(ERROR_CODE_MISSION_ID_NOT_EXIST, "task id is", mission_id);
    }

    auto mission_instance = make_shared<MissionInstance>();
    mission_instance->session_id_ = session_id;
    mission_instance->user_name_ = user_name;
    mission_instance->no_ = no;
    mission_instance->seq_ = seq;
    mission_instance->cur_step_id_ = step_id;
    mission_instance->avoid_policy_ = policy;
    mission_instance->state_ = MissionState::PENDING;
    mission_instance->mission_info_ = mission_info;
    mission_instance->start_timestamp_ = sros::core::util::get_timestamp_in_ms();
    mission_instance->finish_timestamp_ = mission_instance->start_timestamp_;

    bool ret = mission_manager_->enqueueMission(mission_instance);

    LOGGER(INFO, TASK) << "Push task " << mission_info.name_ << " in queue";

    if (!ret) {
        throw EXEC_ERROR(ERROR_CODE_MISSION_ENQUEUE_TIMEOUT, "task id is", mission_id);
    }

    storage_.createMission(mission_id, no, user_name, step_id);
}

AbstractStepPtr MissionModule::getExecMissionStep(bool lastStepSuccess) {
    LOG(INFO) << "Get task next step";

    MissionInstancePtr cur_mission_ptr = getCurrentMission();

    // 查找下一个process,直到查找到process(或end)为止
    AbstractStepPtr cur_mission_step = nullptr;
    bool has_find_step = false;
    int max_loop  = 0;
    while (!has_find_step) {

        // 异常保护
        max_loop++;
        if (max_loop > 100) {
            break;
        }

        // 若上一次执行结果失败，并且下一步不是处理上一步执行结果的decision，就直接结束程序
        cur_mission_step = cur_mission_ptr->getMissionStep(getCurStepId());

        if (cur_mission_step == nullptr) {
            LOG(ERROR) << "NULL task step";
            break;
        }

        MissionStepType step_type = cur_mission_step->getStepType();
        switch (step_type) {
            case MissionStepType::StepDecision: {
                StepDecisionPtr decision_ptr = std::dynamic_pointer_cast<StepDecision>(cur_mission_step);
                if (!decision_ptr) {
                    break;
                }
                DecisionType decision_type = decision_ptr->getDecisionType();
                switch (decision_type) {
                    case DecisionType::DecisionLoop: {
                        decision_ptr->setCurLoopTime(decision_ptr->getCurLoopTime() + 1);
                        LOGGER(INFO, TASK) << "Current loop times：" << decision_ptr->getCurLoopTime();
                        cur_mission_ptr->total_cycle_time_ = decision_ptr->getLoopTimes();
                        cur_mission_ptr->finish_cycle_time_ = decision_ptr->getCurLoopTime();

                        if (decision_ptr->getCurLoopTime() >= decision_ptr->getLoopTimes()) {  // 完成循环
                            cur_mission_ptr->finish_cycle_time_ = 0;  // 复位为0，以免影响其他循环任务
                            cur_mission_ptr->total_cycle_time_ = 0;
                            decision_ptr->setCurLoopTime(0);
                            goNextStep();
                        } else {  // 未完成循环
                            goNextStep(false);
                        }
                        break;
                    }
                    case DecisionType::DecisionReg: {
                        ModbusAddrType register_type = decision_ptr->getRegisterType();
                        uint16_t absolute_addr = reg_admin_->absoluteAddr(decision_ptr->getRegisterAddr(), register_type);
                        uint16_t data = decision_ptr->getRegisterData();
                        ExpressionType expression = decision_ptr->getExpression();
                        // ===================================//
                        // 兼容Matrix1.7.0及之前版本
                        if (register_type == ModbusAddrType::AddrNone) {
                            register_type = reg_admin_->getRegisterAddrType(absolute_addr);
                        }
                        // ===================================//

                        LOGGER(INFO, TASK) << "Execute task step: register condition, addr: " << absolute_addr << " data: " <<
                                  data << " expression: " << static_cast<int>(expression);

                        uint32_t reg_dat = 0;
                        if (!reg_admin_->doReadRegister(absolute_addr, reg_dat, register_type)) {
                            return nullptr;
                        }

                        if (decision_ptr->isRegDatMatch(reg_dat)) {
                            LOGGER(INFO, TASK) << "Register data match, go YES branch, register=" << reg_dat;
                            goNextStep();
                        } else {
                            LOGGER(INFO, TASK) << "Register data mismatch: go NO branch, register=" << reg_dat;
                            goNextStep(false);
                        }
                        break;
                    }
                    case DecisionType::DecisionLastStepResult: {
                        LOG(INFO) << "MissionModule::getExecMissionStep(): Decision Last Step Result";
                        goNextStep(lastStepSuccess);
                        break;
                    }
                    default: {
                        LOG(ERROR) << "Unsupport decision type: " + std::to_string(static_cast<int>(decision_type));
                        return nullptr;
                    }
                }
                break;
            }
            case MissionStepType::StepBegin:
            case MissionStepType::StepEnd:
            case MissionStepType::StepMoveToStation:
            case MissionStepType::StepAction:
            case MissionStepType::StepSetReg:
            case MissionStepType::StepParallel:
            case MissionStepType::StepWaitRegUpdate:
            case MissionStepType::StepWait:
            case MissionStepType::StepPause:
            case MissionStepType::StepMission: {
                has_find_step = true;
                //                LOG(INFO) << "MissionModule::getExecMissionStep(): " << static_cast<int>(step_type);
                break;
            }
            default: {
                LOG(ERROR) << "Unsupport step type: " + std::to_string(static_cast<int>(step_type));
                break;
            }
        }  // end switch
    }      // end while
    return cur_mission_step;
}

std::string MissionModule::getStartStepId() {
    return getCurrentMission()->getStartStepId();
}

std::string MissionModule::getNextStepId(const std::string &cur_step_id, bool decision) {
    // 未处理获取不到的情况
    if (cur_step_id == "") {
        return "";
    }

    return getCurrentMission()->getNextStepId(cur_step_id, decision);
}

bool MissionModule::executeNextStep(bool lastStepSuccess) {
    LOG(INFO) << "Execute next task step";
    goNextStep();
    AbstractStepPtr mission_step = getExecMissionStep(lastStepSuccess);
    return doExecuteStep(mission_step);
}

bool MissionModule::doExecuteStep(AbstractStepPtr mission_step) {
    if (!mission_step) {
        LOG(ERROR) << "Failed to get task step";
        return false;
    }

    auto cur_mission = getCurrentMission();
    if (!cur_mission) {
        LOG(ERROR) << "Null mission pointer";
        return false;
    }

    storage_.updateMissionCurrentStep(cur_mission->no_, mission_step->getStepId());

    // 执行一步
    switch (mission_step->getStepType()) {
        // 结点begin无实际意义，如果时begin结点，则直接进入下一个结点
        case MissionStepType::StepBegin: {
            if (!executeNextStep()) {
                //                execNextMission();
            }
            break;
        }
        case MissionStepType::StepEnd: {
            // 如果父mission队列为空，则结束任务
            if (!mission_manager_->hasParentMission()) {
                getCurrentMission()->result_ = MissionResult::MISSION_RESULT_OK;
                stopMission();
                LOGGER(INFO, TASK) << "Task " << curMissionInfo().name_ << " run finished";
                execNextMission();
            } else {
                mission_manager_->popParentMission();

                // TODO 子任务执行失败的条件分支
                executeNextStep();
            }

            break;
        }
        case MissionStepType::StepMoveToStation: {
            doExecuteStepMoveToStation(mission_step);
            break;
        }
        case MissionStepType::StepAction: {
            doExecuteStepAction(mission_step);
            break;
        }
        case MissionStepType::StepWait: {
            doExecuteStepWait(mission_step);
            break;
        }
        case MissionStepType::StepPause: {
            doExecuteStepPause(mission_step);
            break;
        }
        case MissionStepType::StepSetReg: {
            doExecuteStepSetReg(mission_step);
            break;
        }
        case MissionStepType::StepWaitRegUpdate: {
            doExecuteStepWaitRegUpdate(mission_step);
            break;
        }
        case MissionStepType::StepMission: {
            doExecuteStepChildMission(mission_step);
            break;
        }
        case MissionStepType::StepParallel: {
            doExecuteStepParallel(mission_step);
            break;
        }
        default: {
            LOG(ERROR) << "Unsupport task step: " + std::to_string(static_cast<int>(mission_step->getStepType()));
            break;
        }
    }

    //    LOG(INFO) << "Next Step Info: " << cur_mission_->getNextStepInfo().dump();
    return true;
}

bool MissionModule::doExecuteStepMoveToStation(AbstractStepPtr step) {
    StepMovePtr movePtr = std::dynamic_pointer_cast<StepMove>(step);
    if (!movePtr) {
        return false;
    }

    try {
        g_state.checkStartMovementTaskCondition();
    } catch (const ExecError &e) {
        string err_step_id = getCurStepId();
        mission_manager_->setToRootMission();
        getCurrentMission()->result_ = MissionResult::MISSION_RESULT_FAILED;
        getCurrentMission()->cur_step_id_ = err_step_id;
        getCurrentMission()->err_code_ = e.errorCode();
        stopMission();
        return false;
    }

    LOGGER(INFO, TASK) << "Execute task step: move to station " << movePtr->getStationNo();

    std::deque<sros::core::StationNo_t> dst_stations;
    dst_stations.push_back(movePtr->getStationNo());

    MissionInstancePtr cur_mission_ptr = getCurrentMission();

    auto new_task =
        std::make_shared<sros::core::MovementTask>(0, getName(), dst_stations, cur_mission_ptr->avoid_policy_);
    new_task->setTaskSeq(cur_mission_ptr->seq_);
    new_task->setTaskSessionId(cur_mission_ptr->session_id_);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->req_seq = cur_mission_ptr->seq_;
    mm->session_id = cur_mission_ptr->session_id_;
    mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
    mm->movement_task = new_task;
    sendMsg(mm);
    return true;
}

bool MissionModule::doExecuteStepAction(AbstractStepPtr step) {
    StepActionPtr actionPtr = std::dynamic_pointer_cast<StepAction>(step);
    if (!actionPtr) {
        return false;
    }

    try {
        g_state.checkStartActionTaskCondition(
            actionPtr->getActionId(),
            actionPtr->getActionParam0(),
            actionPtr->getActionParam1()
        );
    } catch (const ExecError &e) {
        string err_step_id = getCurStepId();
        mission_manager_->setToRootMission();
        getCurrentMission()->result_ = MissionResult::MISSION_RESULT_FAILED;
        getCurrentMission()->cur_step_id_ = err_step_id;
        getCurrentMission()->err_code_ = e.errorCode();
        stopMission();
        return false;
    }

    LOGGER(INFO, TASK) << "Execute task step: run action task, action id: " << actionPtr->getActionId()
                          << " action param0: " << actionPtr->getActionParam0()
                          << " action param1: " << actionPtr->getActionParam1();

    auto new_task = std::make_shared<sros::core::ActionTask>(
        0, getName(), actionPtr->getActionId(), actionPtr->getActionParam0(), actionPtr->getActionParam1());

    new_task->setTaskSeq(getCurrentMission()->seq_);
    new_task->setTaskSessionId(getCurrentMission()->session_id_);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->command = sros::core::CMD_NEW_ACTION_TASK;
    mm->param0 = new_task->getTaskNo();
    mm->action_task = new_task;
    mm->session_id = getCurrentMission()->session_id_;

    sendMsg(mm);

    return true;
}

bool MissionModule::doExecuteStepWait(AbstractStepPtr step) {
    StepWaitTimePtr wait_ptr = std::dynamic_pointer_cast<StepWaitTime>(step);
    if (!wait_ptr) {
        return false;
    }

    int sleep_time = wait_ptr->getWaitTime();

    LOGGER(INFO, TASK) << "Execute task step: wait time " << sleep_time;

    // 定时任务在onTimer200ms处理
    //    std::thread t([this, sleep_time]() {
    //        boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_time * 1000));
    //        // 任务被取消或暂停，定时时间到后不会执行
    //        if (getCurrentMission()->state == MissionState::PAUSED ||
    //            getCurrentMission()->state == MissionState::FINISHED) {
    //            return true;
    //        }
    //        if (!executeNextStep()) {
    //            execNextMission();
    //        }
    //    });
    //    t.detach();
    return true;
}

bool MissionModule::doExecuteStepPause(AbstractStepPtr step) {
    LOGGER(INFO, TASK) << "Execute task step: pause";
    handlePauseMovement(NULL);
    return true;
}

bool MissionModule::doExecuteStepSetReg(AbstractStepPtr step) {
    StepSetRegPtr set_reg_ptr = std::dynamic_pointer_cast<StepSetReg>(step);
    if (!set_reg_ptr) {
        return false;
    }

    ModbusAddrType register_type = set_reg_ptr->getRegisterType();
    uint16_t addr = reg_admin_->absoluteAddr(set_reg_ptr->getRegisterAddr(), register_type);
    uint16_t data = set_reg_ptr->getRegisterData();

    // =============================================================//
    // Matrix1.7.0及以前版本，此时register_type=ModbusAddrType::AddrNone,
    if (register_type == core::ModbusAddrType::AddrNone) {
        register_type = reg_admin_->getRegisterAddrType(addr);
    }
    // =============================================================//

    LOGGER(INFO, TASK) << "Execute task step: set register, addr: " << addr << " data: " << data;

    auto mm = make_shared<sros::core::ModbusRegisterMsg>("MODBUS_REGISTER_ACTION");
    mm->operation_type_ = sros::core::RegisterAction::ActionWrite;
    mm->registerType_ = (uint16_t) (register_type);
    mm->addr_ = addr;
    mm->dat_ = data;

    // TODO 发送写寄存器的消息后直接认为结束，暂不考虑寄存器写失败的情形
    sendMsg(mm);

    // 等待消息处理完毕再执行下一个步骤
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    if (!executeNextStep()) {
        return false;
        //        execNextMission();
    }
    return true;
}

bool MissionModule::doExecuteStepWaitRegUpdate(AbstractStepPtr step) {
    // 在定时器中处理
    StepWaitRegUpdatePtr wait_reg_update_ptr = std::dynamic_pointer_cast<StepWaitRegUpdate>(step);
    if (!wait_reg_update_ptr) {
        return false;
    }

    uint16_t absolute_addr = reg_admin_->absoluteAddr(wait_reg_update_ptr->getRegisterAddr(), wait_reg_update_ptr->getRegisterType());
    LOGGER(INFO, TASK) << "Execute task step: Wait for register update, addr: "
              << absolute_addr << " data: " << wait_reg_update_ptr->getRegisterData() << " expression: "
              << std::to_string(static_cast<int> (wait_reg_update_ptr->getExpression()));
    return true;
}

bool MissionModule::doExecuteStepChildMission(AbstractStepPtr step,  const std::string &from_step) {

    auto mission_instance = prepareChildMission(step, from_step);
    if (!mission_instance) {
        return false;
    }

    LOGGER(INFO, TASK) << "Execute task step: run child task " << mission_instance->getName();

    startMission(mission_instance);

    return true;
}

MissionInstancePtr MissionModule::prepareChildMission(AbstractStepPtr step, const std::string &from_step) {
    auto mission_instance = createMissionInstanceFromChildMissionStep(step, from_step);
    if (!mission_instance) {
        return nullptr;
    }

    // 启动子任务时需要将父任务保存
    mission_manager_->pushParentMission(getCurrentMission());

    // 等待200ms确保当前步骤已经上传到前端，新步骤会马上覆盖子任务步骤id，导致前端显示步骤不同步
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    mission_manager_->setCurrentMission(mission_instance);

    return mission_instance;
}

MissionInstancePtr MissionModule::createMissionInstanceFromChildMissionStep(AbstractStepPtr step, const string &start_step_id) {
    StepMissionPtr child_mission = std::dynamic_pointer_cast<StepMission>(step);
    if (!child_mission) {
        return nullptr;
    }

    uint32_t mission_id = child_mission->getMissionId();

    MissionInfo mission_info;
    if (!mission_manager_->loadMissionFromStorage(mission_id, mission_info)) {
        LOG(ERROR) << "Mission not exist in storage: " << mission_id;
        return nullptr;
    }

    MissionInstancePtr cur_mission_ptr = getCurrentMission();
    if (!cur_mission_ptr) {
        LOG(ERROR) << "Parent mission is NULL";
        return nullptr;
    }

    auto mission_instance = make_shared<MissionInstance>();
    mission_instance->isChildMission = true;
    mission_instance->session_id_ = cur_mission_ptr->session_id_;
    mission_instance->user_name_ = cur_mission_ptr->user_name_;
    mission_instance->no_ = cur_mission_ptr->no_;  // 子任务和父任务使用同一个id
    mission_instance->seq_ = cur_mission_ptr->seq_;
    mission_instance->cur_step_id_ = start_step_id;
    mission_instance->avoid_policy_ = cur_mission_ptr->avoid_policy_;
    mission_instance->state_ = MissionState::RUNNING;
    mission_instance->mission_info_ = mission_info;
    mission_instance->start_timestamp_ = sros::core::util::get_timestamp_in_ms();
    mission_instance->finish_timestamp_ = mission_instance->start_timestamp_;

    return mission_instance;
}

bool MissionModule::doExecuteStepParallel(AbstractStepPtr step) {
    StepParallelPtr parallelStep = std::dynamic_pointer_cast<StepParallel>(step);
    if (!parallelStep) {
        return false;
    }

    parallel_action_has_started_ = false;
    movement_task_result_ = TaskResult::TASK_RESULT_NA;
    action_task_result_ = TaskResult::TASK_RESULT_NA;

    LOGGER(INFO, TASK) << "Execute task step: Parallel Step";

    AbstractStepPtr move_step = parallelStep->getStep(MissionStepType::StepMoveToStation);
    if (move_step) {
        doExecuteStepMoveToStation(move_step);
    }
    return true;
}

bool MissionModule::isRunning() const {
    if (!getCurrentMission()) {
        return false;
    }
    return (getCurrentMission()->state_ == MissionState::RUNNING || getCurrentMission()->state_ == MissionState::PAUSED);
}

uint64_t MissionModule::getCurMissionNo() const {
    if (!getCurrentMission()) {
        return 0;
    }
    return getCurrentMission()->no_;
}

std::string MissionModule::getCurStepId() const {
    if (!getCurrentMission()) {
        return "";
    }
    return getCurrentMission()->currentStepId();
}

MissionState MissionModule::getMissionState() const {
    if (!getCurrentMission()) {
        return MissionState::NA;
    }
    return getCurrentMission()->state_;
}

void MissionModule::setCurStepId(const std::string &stepId) {
    if (!getCurrentMission()) {
        return;
    }

    if (stepId == "") {
        return;
    }
    getCurrentMission()->setCurStepId(stepId);
}

void MissionModule::handleNewMission(core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    auto mission_id = msg->mission_id;
    auto mission_cur_step_id = msg->mission_cur_step_id;
    auto no = msg->mission_no;
    auto avoid_policy = msg->mission_avoid_policy;

    LOGGER(INFO, CMD_HANDER) << "Start task " << mission_manager_->getMissionName(mission_id) << ", task id: " << no;

    try {
        // [perry 2022/05/31]删除该处检测，在步骤中做检测处理即可
        //g_state.checkStartMovementTaskCondition();

        // 当前正在执行任务，但是不是mission任务，如单步调试或FMS任务
        if (!isRunning() && TaskManager::getInstance()->isTaskRunning()) {
            if (TaskManager::getInstance()->isMovementTaskRunning()) {
                throw EXEC_ERROR(ERROR_CODE_MOVEMENT_PRE_TASK_RUNNING,
                                 "previous movement task is running, new task is ignored");
            } else {
                throw EXEC_ERROR(ERROR_CODE_ACTION_PRE_TASK_RUNNING,
                                 "previous action task is running, new task is ignored");
            }
        }
        enqueueMission(msg->session_id, no, msg->req_seq, mission_id, mission_cur_step_id, msg->user_name,
                       avoid_policy);
    } catch (const ExecError &e) {
        msg->result_state = RESPONSE_FAILED;
        msg->result_code = e.errorCode();
        responseCommand(msg);
        return;
    }

    // 先回response
    msg->result_state = RESPONSE_OK;
    responseCommand(msg);

    notifyMissionListChanged();
    if ((mission_manager_->pendingMissionSize() == 1) && !isRunning()) {
        execNextMission();
    }
}

void MissionModule::handleCancelMission(core::base_msg_ptr m) {
    // 取消任务
    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    auto mission_no = msg->mission_no;

    LOGGER(INFO, CMD_HANDER) << "Cancel task, task id " << mission_no;

    bool ret = cancelMission(mission_no);

    msg->result_state = RESPONSE_OK;
    msg->result_code = ERROR_CODE_NONE;
    responseCommand(msg);

    // 如果任务没有在执行中，则直接返回
    if (!ret) {
        return;
    }

    // 如果任务正在执行,则通知main module取消任务
    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->req_seq = msg->req_seq;
    mm->session_id = msg->session_id;
    mm->command = sros::core::CMD_COMMON_CANCEL;
    sendMsg(mm);
}

void MissionModule::handlePauseMovement(core::base_msg_ptr m) {
    if (isRunning()) {
        LOGGER(INFO, CMD_HANDER) << "Pause task";
        auto cur_mission = getCurrentMission();
        cur_mission->state_ = MissionState::PAUSED;
        // 子任务和父任务任务共享同一个no
        storage_.updateMissionState(cur_mission->no_, MissionState::PAUSED);
    }
}

void MissionModule::handleContinueMovement(core::base_msg_ptr m) {
    LOGGER(INFO, CMD_HANDER) << "Continue task";
    if (!isRunning()) {
        return;
    }

    auto cur_mission = getCurrentMission();
    if (getMissionState() == MissionState::PAUSED) {
        cur_mission->state_ = MissionState::RUNNING;
        storage_.updateMissionState(cur_mission->no_, MissionState::RUNNING);
    }

    auto missionStep = cur_mission->getMissionStep(getCurStepId());
    if (!missionStep) {
        LOGGER(INFO, CMD_HANDER) << "Invalid task step";
        return;
    }

    // 如果当前步骤是暂停步骤，则继续执行下一步骤
    if (missionStep->getStepType() == MissionStepType::StepPause) {
        LOGGER(INFO, CMD_HANDER) << "Continue task after pause";
        executeNextStep();
    }
}

void MissionModule::handleContinueMission(core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    msg->result_state = RESPONSE_OK;
    msg->result_code = ERROR_CODE_NONE;
    responseCommand(msg);

    manual_stop_ = false;
    execNextMission();
}

void MissionModule::handleReorderMission(core::base_msg_ptr m) {
    LOGGER(INFO, CMD_HANDER) << "Reorder task";

    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    mission_manager_->reorderMissionList(msg->mission_no_lst);

    msg->result_state = RESPONSE_OK;
    msg->result_code = ErrorCode::ERROR_CODE_NONE;

    responseCommand(msg);

    // 重新排序后通知前端更新顺序
    notifyMissionListChanged();
}

void MissionModule::notifyMissionListChanged() {
    // 发送Notification
    auto notify_msg = std::make_shared<sros::core::NotificationMsg>("TOPIC_NOTIFY");
    notify_msg->notify_type = sros::core::NotificationMsg::NOTIFY_MISSION_LIST_CHANGED;
    notify_msg->mission_list = mission_manager_->allMissions();
    sendMsg(notify_msg);
}

void MissionModule::goNextStep(bool decision) {
    std::string step_id = getNextStepId(getCurStepId(), decision);
    if (step_id == "") {
        LOG(ERROR) << "MissionModule::goNextStep(): Next task step is NULL";
        return;
    }
    setCurStepId(step_id);
}

}  // namespace sros
