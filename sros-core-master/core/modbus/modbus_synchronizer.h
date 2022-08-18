//
// Created by caoyan on 6/22/21.
//

#ifndef SROS_MODBUS_SYNCHRONIZER_H
#define SROS_MODBUS_SYNCHRONIZER_H

#include <thirty-party/libmodbus/modbus.h>
#include "register_admin.h"

namespace sros {
namespace modbus {

enum Modbus_Type {
    MODBUS_TCP,
    MODBUS_TCP_PI,
    MODBUS_RTU,
};

class ModbusSynchronizer {
 public:
    ModbusSynchronizer();

    void setReplaceInputRegistersWithHoldRegisters(bool replace_input_registers_with_hold);

    void setEnableExtendFunc(bool enable_extend_func);

    void setBasisInputRegister(uint16_t local_basis_input_register_addr, uint16_t remote_basis_hold_register_addr,
                               uint16_t remote_basis_hold_register_count);

    void setBasisHoldRegister(uint16_t local_basis_hold_register_addr, uint16_t remote_basis_input_register_addr,
                              uint16_t remote_basis_input_register_count);

    void setExtendInputRegister(uint16_t local_extend_input_register_addr, uint16_t remote_extend_hold_register_addr,
                                uint16_t remote_extend_hold_register_count);

    void setExtendHoldRegister(uint16_t local_extend_hold_register_addr, uint16_t remote_extend_input_register_addr,
                               uint16_t remote_extend_input_register_count);

    void setRemoteTcpInfo(const std::string &ip, int port = 502);
    void setRemoteRtuInfo(const std::string &device_name, int baud_rate = 115200);

    void setSlaveId(int slave_id);

    bool connected() const { return connected_; }
    bool synchronizer_ok() const { return synchronizer_ok_; }

    void run();

    std::string getRemoteInfo() const;

    bool isRunning() const { return state_ == RUN; }

 private:
    enum State {
        NONE = 0,
        STOP = 1,
        RUN = 2,
    };
    bool init();
    void loop();
    bool synchronize();

    State state_ = NONE;

    Modbus_Type use_backend_;  // 通信方式

    int slave_id_ = 1;

    std::string device_name_rtu_;   // 串口名
    int baud_rate_ = 115200;        // 波特率
    int port_ = 502;                // 端口号
    std::string ip_ = "127.0.0.1";  // 远程ip
    modbus_t *ctx_ = NULL;
    core::RegisterAdmin *reg_admin_ = core::RegisterAdmin::getInstance();
    bool connected_ = false;  // 标记是否连上
    bool synchronizer_ok_ = false; // 标记是否同步成功

    //基础寄存器
    uint16_t local_basis_input_register_addr_ = 0;
    uint16_t remote_basis_hold_register_addr_ = 0;
    uint16_t remote_basis_hold_register_count_ = 0;

    uint16_t local_basis_hold_register_addr_ = 0;
    uint16_t remote_basis_input_register_addr_ = 0;
    uint16_t remote_basis_input_register_count_ = 0;

    //扩展寄存器
    uint16_t local_extend_input_register_addr_ = 0;
    uint16_t remote_extend_hold_register_addr_ = 0;
    uint16_t remote_extend_hold_register_count_ = 0;

    uint16_t local_extend_hold_register_addr_ = 0;
    uint16_t remote_extend_input_register_addr_ = 0;
    uint16_t remote_extend_input_register_count_ = 0;

    bool enable_extend_func_ = false;
    bool replace_input_registers_with_hold_ = false;
};
}  // namespace modbus
}  // namespace sros

#endif  // SROS_MODBUS_SYNCHRONIZER_H
