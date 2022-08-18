//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_PARALLEL_H
#define SROS_STEP_PARALLEL_H

#include "abstract_step.h"

namespace sros {
namespace core {

using namespace std;
class StepParallel : public AbstractStep {
public:
    explicit StepParallel();
    virtual ~StepParallel();

    vector<AbstractStepPtr> &get_all_steps() { return steps_; }
    AbstractStepPtr getStep (MissionStepType step_type);

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    vector<AbstractStepPtr> steps_; // 并行任务所有步骤
};
typedef std::shared_ptr<StepParallel> StepParallelPtr;
}
}

#endif //SROS_STEP_PARALLEL_H
