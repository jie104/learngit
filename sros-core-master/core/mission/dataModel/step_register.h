//
// Created by huangwuxian on 19-8-6.
//

#ifndef SROS_STEP_REGISTER_H
#define SROS_STEP_REGISTER_H

#include "abstract_step.h"
#include "../../modbus/register_admin.h"

namespace sros {
namespace core {

class StepRegister : public AbstractStep {
public:
    explicit StepRegister(MissionStepType eType);
    virtual ~StepRegister();

    inline uint16_t getRegisterAddr() const { return reg_addr_; }
    inline uint16_t getRegisterData() const { return  reg_data_; }
    inline ModbusAddrType getRegisterType() const { return modbus_register_type_; };

protected:
    // 如果该字段为ModbusAddrType::AddrNone
    // 表示Matrix1.7.0及之前版本，寄存器addr_为绝对地址
    // 否则addr_为相对地址
    ModbusAddrType modbus_register_type_;

    uint16_t reg_addr_;
    uint16_t reg_data_;
};
typedef std::shared_ptr<StepRegister> StepRegisterPtr;

}
}


#endif //SROS_STEP_REGISTER_H
