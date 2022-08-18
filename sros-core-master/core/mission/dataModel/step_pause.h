//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_PAUSE_H
#define SROS_STEP_PAUSE_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepPause : public AbstractStep {
public:
    explicit StepPause();
    virtual ~StepPause();

private:

};
typedef std::shared_ptr<StepPause> StepPausePtr;
}
}

#endif //SROS_STEP_PAUSE_H
