//
// Created by lhx on 17-1-12.
//

#include "task.h"

namespace sros {
namespace core {

Task::Task(TaskNo_t no, const std::string &source_module)
    : seq_(0), no_(no), state_(TASK_NA), result_(TASK_RESULT_NA), source_module_(source_module) {
    static int g_task_no = 1;  // 全局任务编号

    if (no == 0) {
        no_ = g_task_no;
    }

    g_task_no += 1;
}

Task::~Task() {}

uint32_t Task::getTaskSeq() const { return seq_; }

uint64_t Task::getTaskSessionId() const { return session_id_; }

void Task::setTaskSeq(uint32_t seq) { seq_ = seq; }

void Task::setTaskSessionId(uint64_t session_id) { session_id_ = session_id; }

TaskNo_t Task::getTaskNo() const { return no_; }

const TaskState &Task::getState() const { return state_; }

void Task::updateState(const TaskState &state) { state_ = state; }

void Task::startTask() {
    if (state_ == TASK_NA || state_ == TASK_WAIT_FOR_START) {
        state_ = TASK_RUNNING;
    }
}

bool Task::isRunning() const {
    return (state_ != TASK_FINISHED && state_ != TASK_NA);  // 除了任务结束，和任务不可用的状态，其他状态任务都在运行中
}

void Task::finishTask(TaskResult result) {
    state_ = TASK_FINISHED;
    result_ = result;
}

bool Task::isFinished() const { return (state_ == TASK_FINISHED || state_ == TASK_NA); }

TaskResult Task::getTaskResult() const { return result_; }

bool Task::isWaitForStart() const { return (state_ == TASK_WAIT_FOR_START); }

bool Task::isInCancel() const { return (state_ == TASK_IN_CANCEL); }

}  // namespace core
}  // namespace sros
