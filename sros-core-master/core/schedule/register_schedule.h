//
// Created by huangwuxian on 20-3-9.
//

#ifndef SROS_REGISTER_SCHEDULE_H
#define SROS_REGISTER_SCHEDULE_H

#include "abstrace_schedule.h"
#include "core/mission/constant.h"
#include "core/modbus/register_admin.h"

namespace sros {
namespace core {

class RegisterSchedule : public AbstractSchedule {
public:
    explicit RegisterSchedule();
    virtual ~RegisterSchedule();

    inline uint16_t getRegisterAddr() const { return addr_; }
    inline uint16_t getRegisterValue() const { return value_; }
    inline ModbusAddrType getRegisterType() const { return type_; }
    inline ExpressionType getExpression() const { return expression_; }

    inline int32_t getRegisterLastValue() const { return register_last_value_; }
    inline void setRegisterLastValue(uint32_t val) { register_last_value_ = val; }
    bool isRegisterValueSatisfied(uint32_t register_value);

protected:
    virtual bool loadDetailsFromJson(const nlohmann::json &dat) override;

private:
    int32_t register_last_value_; // 记录寄存器上次读取到的值,只有出现跳变时才需要触发任务执行

    uint16_t addr_; // 寄存器地址
    uint16_t value_; // 寄存器值

    ModbusAddrType type_; // 寄存器类型
    ExpressionType expression_; // 比较表达式
};

typedef std::shared_ptr<RegisterSchedule> RegisterSchedulePtr;
}
}



#endif //SROS_REGISTER_SCHEDULE_H
