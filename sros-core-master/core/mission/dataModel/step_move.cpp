//
// Created by huangwuxian on 19-3-1.
//

#include "step_move.h"

namespace sros {
namespace core {

StepMove::StepMove() : AbstractStep(MissionStepType::StepMoveToStation), station_no_(0) {

}

StepMove::~StepMove() {

}

bool StepMove::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepMove::parseStepInfo(): Invalid json dat " << step_info;
        return false;
    }
    station_no_ = step_info.at("stationId").get<int>();
    return true;
}

}
}

