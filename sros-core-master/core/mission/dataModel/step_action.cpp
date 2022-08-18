//
// Created by huangwuxian on 19-3-1.
//

#include "step_action.h"

namespace sros {
namespace core {

StepAction::StepAction() : AbstractStep(MissionStepType::StepAction),
action_id_(0),
action_param0(0),
action_param1(0) {

}

StepAction::~StepAction() {

}

bool StepAction::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepAction::parseStepInfo(): Invalid json dat " << step_info;
        return false;
    }
    action_id_ = step_info.at("actionId");
    action_param0 = step_info.at("actionParameter");
    action_param1 = step_info.at("actionParameter1");
    return true;
}

}
}