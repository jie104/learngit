//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_WAIT_TIME_H
#define SROS_STEP_WAIT_TIME_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepWaitTime : public AbstractStep {
public:
    explicit StepWaitTime();
    virtual ~StepWaitTime();

    inline int getWaitTime() const { return wait_time_; }

    inline int getRemainTime() const { return remain_time_; }
    inline void setRemainTime(int value) { remain_time_ = value; }

    // 重置剩余时间
    inline void resetRemainTime() { remain_time_ = wait_time_; }

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    int wait_time_;   // 单位200ms，以使用系统定时器
    int remain_time_; // 剩余时间，定时过程中可能暂停，此时记录定时剩下时间
};
typedef std::shared_ptr<StepWaitTime> StepWaitTimePtr;
}
}

#endif //SROS_STEP_WAIT_TIME_H
