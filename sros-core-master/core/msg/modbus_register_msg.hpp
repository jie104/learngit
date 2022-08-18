//
// Created by huangwuxian on 19-3-7.
//

#ifndef SROS_MODBUS_REGISTER_MSG_H
#define SROS_MODBUS_REGISTER_MSG_H

#include "core/msg/base_msg.h"

namespace sros {
namespace core {

enum class RegisterAction {
    ActionNone = 0,
    ActionRead,
    ActionWrite,
};

class ModbusRegisterMsg : public BaseMsg {
public:
    ModbusRegisterMsg (const topic_t &topic) : BaseMsg(topic, TYPE_MODBUS_REGISTER),
    uuid_(util::get_timestamp_in_ms()), addr_(0), dat_(0), quanlity_(1), registerType_(0),
    operation_type_(RegisterAction::ActionNone) {
    }
    virtual ~ModbusRegisterMsg() {}

    virtual void getTime() {};

    std::string getOperationStr() const {
        std::string str = "None";
        switch (operation_type_) {
            case RegisterAction::ActionWrite: {
                str = "Write";
                break;
            }
            case RegisterAction::ActionRead: {
                str = "Read";
                break;
            }
            default: {
                break;
            }
        }
        return str;
    }

public:
    uint16_t uuid_; // 唯一id

    uint16_t registerType_; // modbus寄存器类型
    uint16_t addr_; // 寄存器地址
    uint32_t dat_;  // 寄存器值
    uint8_t quanlity_; // 寄存器数量, 默认操作单个寄存器

    RegisterAction operation_type_;
};

}
}


#endif //SROS_MODBUS_REGISTER_MSG_H
