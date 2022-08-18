//
// Created by lhx on 17-1-12.
//

#ifndef SROS_ACTION_TASK_H
#define SROS_ACTION_TASK_H

#include "task.h"

namespace sros {
namespace core {

enum LoadingState {
    LOADING_NONE = 0,
    LOADING_FULL = 1,
    LOADING_FREE = 2,
};

class Action {
 public:
    int id;
    int param0;
    int param1;

    int static_param_num = 0;  // 固定参数的个数，有些参数为可变参数用X或Y表示，这样的参数无法做唯一标示

    bool operator<(const Action &rhs) const {
        if (id != rhs.id) {
            return id < rhs.id;
        } else {
            if (param0 != rhs.param0) {
                return param0 < rhs.param0;
            } else {
                return param1 < rhs.param1;
            }
        }
    }
};

class ActionTask : public Task {
 public:
    ActionTask(TaskNo_t no, const std::string &source_module, int id, int param0, int param1, int param2 = 0);

    int getActionID() const;
    int getActionParam() const;
    int getActionParam1() const;
    int getActionParam2() const { return param2_; }
    int getActionResultValue() const;
    std::string getActionParamStr() const { return param_str_; }

    LoadingState getLoadingState() const;

    const std::string &getResultValueStr() const;

    void finishTask(TaskResult result, int result_value);

    void cancelTask();

    virtual int getProcessPercentage() const;

    virtual int getRemainTime() const;

    void setResultValueStr(const std::string &value_str);
    void setLoadingState(LoadingState state);
    void setActionParamStr(const std::string &param_str);

 private:
    int id_;
    int param0_;
    int param1_;
    int param2_;

    std::string param_str_;

    LoadingState loading_state_;

    int result_value_;

    std::string result_value_str_;
};

typedef std::shared_ptr<ActionTask> ActionTask_ptr;

}  // namespace core
}  // namespace sros

#endif  // SROS_ACTION_TASK_H
