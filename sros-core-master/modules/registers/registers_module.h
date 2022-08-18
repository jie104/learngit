/**
 * @file registers_module
 *
 * @author pengjiali
 * @date 2/13/20.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_REGISTERS_MODULE_H
#define SROS_REGISTERS_MODULE_H

#include "core/core.h"
#include "core/modbus/register_admin.h"
#include "core/msg/base_msg.h"
#include "core/src.h"

namespace sros {

class RegistersModule : public sros::core::Module {
 public:
    explicit RegistersModule() :sros::core::Module("RegistersModule") {}
    virtual ~RegistersModule() = default;

    virtual void run();

 private:
    void onTimer_100ms(core::base_msg_ptr m);

    void onTimer_5s(core::base_msg_ptr m);

    // 更新输入寄存器、离散输入寄存器
    void updateStaticInfo(); // 更新一些静态的信息，在程序运行过程中不会改变的；如：版本号
    void updateIPAddress();
    void updateAllDiscreteInputs();
    void updateAllInputRegister();

    // 注意，为了提高批量读写寄存器的速度，默认是不加锁的
    inline void setInputRegister16(uint16_t addr, uint16_t value, bool lock = false);
    inline void setInputRegister32(uint16_t addr, uint32_t value, bool lock = false);
    inline void setInputRegister32(uint16_t addr, int32_t value, bool lock = false);

    void setPoseToRegister(const core::Pose &pose);

    std::string getIPaddress();
    std::string getIPaddressFromFile();
    std::string getIPaddressFromAdapter();

 private:
    core::RegisterAdmin *reg_admin_ = NULL;
    std::string ip_address_ = "0.0.0.0";
};

}  // namespace sros

#endif  // SROS_REGISTERS_MODULE_H
