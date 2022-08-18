//
// Created by huangwuxian on 19-3-11.
//

#ifndef SROS_STEP_MISSION_H
#define SROS_STEP_MISSION_H

#include "abstract_step.h"

namespace sros {
namespace core {

class StepMission : public AbstractStep {
public:
    explicit StepMission();
    virtual ~StepMission();

    uint32_t getMissionId() const { return id_; }

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:
    uint32_t id_; // mission id, 只保存id不保存名字，因为名字可能被修改
};

typedef std::shared_ptr<StepMission> StepMissionPtr;
}
}


#endif //SROS_STEP_MISSION_H
