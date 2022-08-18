//
// Created by huangwuxian on 19-3-2.
//

#ifndef SROS_STEP_BEGIN_H
#define SROS_STEP_BEGIN_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepBegin : public AbstractStep {
public:
    explicit StepBegin();
    virtual ~StepBegin();

public:
};
typedef std::shared_ptr<StepBegin> StepBeginPtr;
}
}

#endif //SROS_STEP_BEGIN_H
