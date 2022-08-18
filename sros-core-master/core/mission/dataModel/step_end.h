//
// Created by huangwuxian on 19-3-2.
//

#ifndef SROS_STEP_END_H
#define SROS_STEP_END_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepEnd : public AbstractStep {
public:
    explicit StepEnd();
    virtual ~StepEnd();

public:

};
typedef std::shared_ptr<StepEnd> StepEndPtr;
}
}


#endif //SROS_STEP_END_H
