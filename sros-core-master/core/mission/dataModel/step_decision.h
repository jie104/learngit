//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_DECISION_H
#define SROS_STEP_DECISION_H

#include "step_register.h"

namespace sros {
namespace core {

class StepDecision : public StepRegister {
public:
    explicit StepDecision();
    virtual ~StepDecision();

    inline DecisionType getDecisionType() const { return decision_type_; };

    inline uint32_t getLoopTimes() const { return loop_times_; }
    inline uint32_t getCurLoopTime() const { return cur_loop_time_; }
    inline void setCurLoopTime(uint32_t loop_time) { cur_loop_time_ = loop_time; }

    inline ExpressionType getExpression() const { return expression_; }

    bool isRegDatMatch(uint16_t req_dat) const;

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    DecisionType decision_type_; // 条件类型

    // 循环次数
    uint32_t loop_times_; // 循环次数
    uint32_t cur_loop_time_; // 记录当前循环正在执行的次数

    ExpressionType expression_;
};
typedef std::shared_ptr<StepDecision> StepDecisionPtr;
}
}




#endif //SROS_STEP_DECISION_H
