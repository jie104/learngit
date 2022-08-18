//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_MOVE_H
#define SROS_STEP_MOVE_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepMove : public AbstractStep {
public:
    explicit StepMove();
    virtual ~StepMove();

    inline uint32_t getStationNo() const { return station_no_; }

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    uint32_t station_no_;
};
typedef std::shared_ptr<StepMove> StepMovePtr;
}
}

#endif //SROS_STEP_MOVE_H
