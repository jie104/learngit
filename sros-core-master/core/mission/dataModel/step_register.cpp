//
// Created by huangwuxian on 19-8-6.
//

#include "step_register.h"

namespace sros {
namespace core {

StepRegister::StepRegister(sros::core::MissionStepType eType) : AbstractStep(eType),
modbus_register_type_(ModbusAddrType::AddrNone),
reg_addr_(0),
reg_data_(0){

}

StepRegister::~StepRegister() {

}

}
}
