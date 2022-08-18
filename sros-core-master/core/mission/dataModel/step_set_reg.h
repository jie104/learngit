//
// Created by huangwuxian on 19-3-1.
//

#ifndef SROS_STEP_SET_REG_H
#define SROS_STEP_SET_REG_H

#include "step_register.h"

namespace sros {
namespace core {

class StepSetReg : public StepRegister {
public:
    explicit StepSetReg();
    virtual ~StepSetReg();

    virtual bool parseStepInfo (const nlohmann::json &step_info) override;

private:

};
typedef std::shared_ptr<StepSetReg> StepSetRegPtr;
}
}

#endif //SROS_STEP_SET_REG_H
