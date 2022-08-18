//
// Created by huangwuxian on 19-3-1.
//

#include "step_decision.h"
namespace sros {
namespace core {

StepDecision::StepDecision() : StepRegister(MissionStepType::StepDecision)
, decision_type_(DecisionType::DecisionNa)
, expression_(ExpressionType::ExpEqual)
, loop_times_(1)
, cur_loop_time_(0) {

}

StepDecision::~StepDecision() {

}

bool StepDecision::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepDecision::parseStepInfo(): Invalid json dat " << step_info;
        return false;
    }

    nlohmann::json decision_info = step_info.at("decisionInfo");
    std::string decision_type = decision_info.at("decisionType").get<std::string>();
    if (decision_type == "loop") {
        decision_type_ = DecisionType::DecisionLoop;
        loop_times_ = decision_info.at("loopTimes").get<int>();
    } else if (decision_type == "regValue") {
        decision_type_ = DecisionType::DecisionReg;
        reg_addr_ = decision_info.at("addr").get<uint16_t>();
        reg_data_ = decision_info.at("value").get<uint16_t>();
        std::string expression = decision_info.at("compare").get<std::string>();
        expression_ = getExpressionType(expression);
        if (decision_info.find("regType") != decision_info.end()) {
            modbus_register_type_ = (ModbusAddrType) (decision_info.at("regType").get<uint16_t>());
        }
    } else if (decision_type == "lastStepResult") {
        decision_type_ = DecisionType::DecisionLastStepResult;
    }
    return true;
}

bool StepDecision::isRegDatMatch(uint16_t req_dat) const {
    if (decision_type_ != DecisionType::DecisionReg) {
        return false;
    }
    switch (expression_) {
        case ExpressionType::ExpEqual: {
            return req_dat == reg_data_;
        }
        case ExpressionType::ExpLarger: {
            return req_dat > reg_data_;
        }
        case ExpressionType::ExpLessThan: {
            return req_dat < reg_data_;
        }
        default: {
            break;
        }
    }
    return false;
}

}
}