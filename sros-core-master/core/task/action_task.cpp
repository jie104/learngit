//
// Created by lhx on 17-1-12.
//

#include "action_task.h"

#include <glog/logging.h>
#include "core/logger.h"

namespace sros {
namespace core {

ActionTask::ActionTask(TaskNo_t no, const std::string &source_module, int id, int param0, int param1, int param2)
        : Task(no, source_module),
          id_(id),
          param0_(param0),
          param1_(param1),
          param2_(param2),
          result_value_(0),
          param_str_("") {

}

int ActionTask::getActionID() const {
    return id_;
}

int ActionTask::getActionParam() const {
    return param0_;
}
int ActionTask::getActionParam1() const {
    return param1_;
}

LoadingState ActionTask::getLoadingState() const {
    return loading_state_;
}

int ActionTask::getActionResultValue() const {
    return result_value_;
}

void ActionTask::finishTask(TaskResult result, int result_value) {
    LOGGER(INFO, ACTION_TASK) << "Action task " << no_ << " finished; id:" << id_
              << ", result:" << result << ", result_value:" << result_value;

    Task::finishTask(result);

    result_value_ = result_value;
}

void ActionTask::cancelTask() {
    LOGGER(INFO, ACTION_TASK) << "Action task " << no_ << " canceled, id:" << id_;

    Task::finishTask(TASK_RESULT_CANCELED);
}

int ActionTask::getProcessPercentage() const {
    return 0;
}

int ActionTask::getRemainTime() const {
    return 0;
}

const std::string &ActionTask::getResultValueStr() const {
    return result_value_str_;
}

void ActionTask::setResultValueStr(const std::string &value_str) {
    result_value_str_ = value_str;
}

void ActionTask::setLoadingState(LoadingState state) {
    loading_state_ = state;
}

void ActionTask::setActionParamStr(const std::string &param_str) {
    param_str_ = param_str;
}

}
}
