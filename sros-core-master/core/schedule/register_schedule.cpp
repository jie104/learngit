//
// Created by huangwuxian on 20-3-9.
//

#include "register_schedule.h"

namespace sros {
namespace core {

RegisterSchedule::RegisterSchedule() : AbstractSchedule(ScheduleType::SCHEDULE_MODBUS)
, addr_(0)
, value_(0)
, register_last_value_(-1)
, type_(ModbusAddrType::AddrNone)
, expression_(ExpressionType::ExpEqual) {

}

RegisterSchedule::~RegisterSchedule() {

}

bool RegisterSchedule::loadDetailsFromJson(const nlohmann::json &dat) {
    if (dat.is_null() || !dat.is_object()) {
        LOG(ERROR) << "RegisterSchedule::loadDetailsFromJson(): Invalid dat info";
        return false;
    }

    addr_ = dat.at("addr").get<uint16_t>();
    value_ = dat.at("value").get<uint16_t>();
    type_ = (ModbusAddrType) dat.at("regType").get<int32_t>();
    expression_ = (ExpressionType) dat.at("expression").get<int32_t>();
    return true;
}

bool RegisterSchedule::isRegisterValueSatisfied(uint32_t register_value) {
    if (register_value == register_last_value_) {
        return false;
    }

    bool result = false;
    switch(expression_) {
        case ExpressionType::ExpLessThan: {
            result = register_value < value_;
            break;
        }
        case ExpressionType::ExpEqual: {
            result = register_value == value_;
            break;
        }
        case ExpressionType::ExpLarger: {
            result = register_value > value_;
            break;
        }
        default: {
            break;
        }
    }
    register_last_value_ = register_value;
    return result;
}

}
}
