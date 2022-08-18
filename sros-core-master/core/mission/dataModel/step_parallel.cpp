//
// Created by huangwuxian on 19-3-1.
//

#include "step_parallel.h"
#include "step_move.h"
#include "step_action.h"

namespace sros {
namespace core {

StepParallel::StepParallel() : AbstractStep(MissionStepType::StepParallel) {

}

StepParallel::~StepParallel() {

}

bool StepParallel::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepMove::fromJson(): Invalid json dat " << step_info;
        return false;
    }

    for (nlohmann::json step : step_info.at("stepList")) {
        auto step_type = step.at("stepType").get<string>();
        AbstractStepPtr ptr_step = nullptr;
        if (step_type == "moveToStation") {
            ptr_step = make_shared<StepMove>();
        } else if (step_type == "action") {
            ptr_step = make_shared<StepAction>();
        }
        ptr_step->parseStepInfo(step);
        steps_.push_back(ptr_step);
    }
    return true;
}

AbstractStepPtr StepParallel::getStep(MissionStepType step_type) {
    for (auto step : steps_) {
        if (step->getStepType() == step_type) {
            return step;
        }
    }
    return nullptr;
}
}
}

