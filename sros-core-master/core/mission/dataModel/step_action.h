//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_ACTION_H
#define SROS_STEP_ACTION_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepAction : public AbstractStep {
public:
    explicit StepAction();
    virtual ~StepAction();

    inline uint32_t getActionId() const { return action_id_; }
    inline uint32_t getActionParam0() const { return action_param0; };
    inline uint32_t getActionParam1() const { return action_param1; }

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    uint32_t action_id_;
    uint32_t action_param0;
    uint32_t action_param1;
};
typedef std::shared_ptr<StepAction> StepActionPtr;
}
}

#endif //SROS_STEP_ACTION_H
