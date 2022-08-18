//
// Created by huangwuxian on 19-3-1.
//

#include "abstract_step.h"

namespace sros {
namespace core {

AbstractStep::AbstractStep(sros::core::MissionStepType eType) : step_type_(eType)
        , step_id_(""){

}

AbstractStep::~AbstractStep() {

}

bool AbstractStep::fromJson(const nlohmann::json &dat) {
    if (dat.is_null() || !dat.is_object()) {
        LOG(ERROR) << "AbstractStep::fromJson(): Invalid json dat " << dat;
        return false;
    }

    this->originalDat_ = dat;

    source_conn_ids_.clear();
    target_conn_ids_.clear();

    step_id_ = dat.at("id").get<std::string>();
    for (std::string srcId : dat.at("sourceConnections")) {
        source_conn_ids_.push_back(srcId);
    }

    for (std::string targetId : dat.at("targetConnections")) {
        target_conn_ids_.push_back(targetId);
    }

    if (step_type_ == MissionStepType::StepNone || step_type_ == MissionStepType::StepBegin ||
    step_type_ == MissionStepType::StepEnd || step_type_ == MissionStepType::StepPause) {
        return true;
    }
    return parseStepInfo(dat.at("stepInfo"));
}

ExpressionType AbstractStep::getExpressionType(const std::string &expression) {
    if (expression == ">") {
        return ExpressionType::ExpLarger;
    } else if (expression == "<") {
        return ExpressionType::ExpLessThan;
    }
    return ExpressionType::ExpEqual;
}

std::string AbstractStep::toString() const {
    if (originalDat_.is_null()) {
        return "";
    }
    return originalDat_.dump();
}

bool AbstractStep::parseStepInfo(const nlohmann::json &step_info) {
    return true;
}

}
}

