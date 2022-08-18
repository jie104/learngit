//
// Created by huangwuxian on 19-3-1.
//

#include "step_set_reg.h"

namespace sros {
namespace core {

StepSetReg::StepSetReg() : StepRegister(MissionStepType::StepSetReg) {

}

StepSetReg::~StepSetReg() {

}

bool StepSetReg::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepSetReg::parseStepInfo(): Invalid json dat " << step_info;
        return false;
    }

    reg_addr_ = step_info.at("addr").get<uint16_t>();
    reg_data_ = step_info.at("value").get<uint16_t>();

    if (step_info.find("regType") != step_info.end()) {
        modbus_register_type_ = (ModbusAddrType) (step_info.at("regType").get<uint16_t >());
    }
    return true;
}
}
}