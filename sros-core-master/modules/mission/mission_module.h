
#ifndef MISSION_MODULE_H
#define MISSION_MODULE_H

#include "core/core.h"
#include "core/state.h"

#include "core/msg/command_msg.hpp"
#include "core/msg/notification_msg.hpp"

#include "core/mission/mission_storage.h"
#include "core/mission/mission_instance.h"
#include "core/mission/mission_manager.h"

#include "core/modbus/register_admin.h"

namespace sros {

using namespace core;
using namespace std;

// 处理mission执行
class MissionModule : public sros::core::Module {
public:
    explicit MissionModule(const std::string &module_name = "MissionModule");
    ~MissionModule();

    virtual void run();

private:
    void execNextMission();

    void onDebugCmdMsg(core::base_msg_ptr m);
    void onCommandResponseMsg(sros::core::base_msg_ptr m);
    void onNotifyMsg(sros::core::base_msg_ptr m);
    void onTimer200ms(sros::core::base_msg_ptr m);

    void handleNewMission(core::base_msg_ptr m);
    void handleCancelMission(core::base_msg_ptr m);
    void handlePauseMovement(core::base_msg_ptr m);

    // 继续当前任务，针对任务执行过程中被暂停或执行到暂停步骤
    void handleContinueMovement(core::base_msg_ptr m);

    // 继续执行下一个任务，任务队列中当前任务被取消后，进入PENDING状态
    // 点击继续按钮时，继续下一个任务
    // TODO 在子任务中恢复执行步骤因改动较大当前未处理
    void handleContinueMission(core::base_msg_ptr m);

    void handleReorderMission(core::base_msg_ptr m);

    void enqueueMission(uint64_t session_id, uint64_t no, uint32_t seq, uint32_t mission_id,
                            std::string step_id, std::string user_name, ObstacleAvoidPolicy policy);

    void responseCommand(const sros::core::CommandMsg_ptr &msg);
    void notifyMissionListChanged();

private:
    bool startMission(MissionInstancePtr mission);
    bool cancelMission(uint64_t mission_no);
    bool stopMission(bool manual_stop = false);

private:
    bool doExecuteStep(AbstractStepPtr mission_step); // 执行一步mission
    bool executeNextStep(bool lastStepSuccess = true);

    bool doExecuteStepMoveToStation(AbstractStepPtr step);
    bool doExecuteStepAction(AbstractStepPtr step);
    bool doExecuteStepWait(AbstractStepPtr step);
    bool doExecuteStepPause(AbstractStepPtr step);
    bool doExecuteStepSetReg(AbstractStepPtr step);
    bool doExecuteStepWaitRegUpdate(AbstractStepPtr step);
    bool doExecuteStepChildMission(AbstractStepPtr step, const std::string &from_step = "");
    bool doExecuteStepParallel(AbstractStepPtr step);

    MissionInstancePtr prepareChildMission(AbstractStepPtr step, const std::string &from_step);
    MissionInstancePtr createMissionInstanceFromChildMissionStep(AbstractStepPtr step, const std::string &start_step_id = "");

    std::string getStartStepId(); // 获取开始节点
    std::string getNextStepId(const std::string &cur_step_id, bool decision = true); // 获取下一个节点，若为decision节点，需要输入参数

    MissionInstancePtr getCurrentMission() const { return mission_manager_->getCurrentRunningMission(); }
    MissionInfo &curMissionInfo() { return getCurrentMission()->mission_info_; }

    bool isRunning() const; // 获取状态

    // 获取真实执行的任务步骤，不包括条件步骤(条件步骤会转为根据条件满足的步骤)
    AbstractStepPtr getExecMissionStep(bool lastStepSuccess = true);

    uint64_t getCurMissionNo() const;

    std::string getCurStepId() const; // NOTE: 此处可能会有脏读但是不会崩溃
    MissionState getMissionState() const;

    void setCurStepId(const std::string &stepId);
    void goNextStep(bool decision = true);
private:
    bool manual_stop_;
    bool parallel_action_has_started_; // 记录并行任务时动作是否已经启动

    // 任务执行结果，主要用于并行任务执行结果判断
    TaskResult movement_task_result_; // 移动任务执行结果
    TaskResult action_task_result_; // 动作任务执行结果

    // 记录上一个系统状态
    SystemState last_sys_state_;

    MissionStorage storage_;
    MissionManager *mission_manager_;

    RegisterAdmin *reg_admin_;
};

} // namespace sros

#endif // MISSION_MODULE_H
