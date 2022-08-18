//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_WAIT_REG_UPDATE_H
#define SROS_STEP_WAIT_REG_UPDATE_H

#include "step_register.h"

namespace sros {
namespace core {

class StepWaitRegUpdate : public StepRegister {
public:
    explicit StepWaitRegUpdate();
    virtual ~StepWaitRegUpdate();

    inline ExpressionType getExpression() const { return expression_; }

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    ExpressionType expression_;
};
typedef std::shared_ptr<StepWaitRegUpdate> StepWaitRegUpdatePtr;
}
}

#endif //SROS_STEP_WAIT_REG_UPDATE_H
