//
// Created by huangwuxian on 19-3-11.
//

#include "step_mission.h"

namespace sros {
namespace core {

StepMission::StepMission() : AbstractStep(MissionStepType::StepMission),
id_(0) {

}

StepMission::~StepMission() {

}

bool StepMission::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepMission::parseStepInfo(): Invalid json dat " << step_info;
        return false;
    }
    id_ = step_info.at("missionId");
    return true;
}

}
}