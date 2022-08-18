//
// Created by caoyan on 6/22/21.
//

#ifndef SROS_SIMPLIFY_MODBUS_ACTION_CONTROLLER_H
#define SROS_SIMPLIFY_MODBUS_ACTION_CONTROLLER_H

#include <string>
#include "core/modbus/register_admin.h"
#include "core/task/task.h"

namespace sros {
namespace eac {

//状态枚举
enum EacSysControl {
    ESC_NONE = 0x00,
    ESC_RESET = 0x01,
    ESC_CLEAR = 0x02,
};

enum EacActControl {
    EAC_NONE = 0x00,
    EAC_START_ACTION = 0x01,
    EAC_PAUSE_ACTION = 0x02,
    EAC_CONTINUE_ACTION = 0x03,
    EAC_CANCEL_ACTION = 0x04,
};

enum EacSysState {
    ESS_NONE = 0x00,
    ESS_INITING = 0x01,
    ESS_ERROR = 0x02,
    ESS_NORMAL = 0x03,
};

enum EacActState {
    EAS_NONE = 0x00,
    EAS_IDLE = 0x01,
    EAS_RUNNING = 0x02,
    EAS_PAUSING = 0x03,
    EAS_ABNORMAL_PAUSING = 0x04,
    EAS_FIN_SUCCESS = 0x05,
    EAS_FIN_FAIL = 0x06,
    EAS_FIN_CANCEL = 0x07,
};

enum AssisFunc {
    AF_BAN_MOVE_TASK = 0x01,
};

enum SrosActState {
    SAS_NONE = 0x00,
    SAS_IDLE = 0x01,
    SAS_RUNNING = 0x02,
    SAS_PAUSING = 0x03,
    SAS_FIN_SUCCESS = 0x04,
    SAS_FIN_FAIL = 0x05,
    SAS_FIN_CANCEL = 0x06,
};

enum SrosSysControl {
    SSC_NONE = 0x00,
    SSC_RESET = 0x01,
};

enum SrosActControl {
    SAC_NONE = 0x00,
    SAC_START_ACTION = 0x01,
    SAC_PAUSE_ACTION = 0x02,
    SAC_CONTINUE_ACTION = 0x03,
    SAC_CANCEL_ACTION = 0x04,
};

//基础功能寄存器地址枚举
enum BasisHoldRegisterAddr {
    BH_ADDR_EAC_SYS_CONTROL = 0,
    BH_ADDR_EAC_ACT_CONTROL = 1,
    BH_ADDR_ACTION_NO = 2,
    BH_ADDR_ACTION_ID = 4,
    BH_ADDR_ACTION_PARAM0 = 5,
    BH_ADDR_ACTION_PARAM1 = 6,
};

enum BasisInputRegisterAddr {
    BI_ADDR_EAC_SYS_STATE = 0,
    BI_ADDR_EAC_SYS_ERROR_CODE = 1,
    BI_ADDR_EAC_ACT_STATE = 2,
    BI_ADDR_EAC_ACT_RESULT = 3,
    BI_ADDR_RUN_ACTION_NO = 5,
    BI_ADDR_LOAD_STATE = 7,
    BI_ADDR_ASSIS_FUNC = 8,
    BI_ADDR_HEART_BEAT = 9,
    BI_ADDR_EAC_SYS_VERSION = 10,
};

//扩展功能寄存器地址枚举
enum ExtendHoldRegisterAddr {
    EH_ADDR_SROS_ACT_STATE = 0,
    EH_ADDR_SROS_ACT_RESULT = 1,
    EH_ADDR_RUN_ACTION_NO = 3,
};

enum ExtendInputRegisterAddr {
    EI_ADDR_SROS_SYS_CONTROL = 0,
    EI_ADDR_SROS_ACT_CONTROL = 1,
    EI_ADDR_ACTION_NO = 2,
    EI_ADDR_ACTION_ID = 4,
    EI_ADDR_ACTION_PARAM0 = 5,
    EI_ADDR_ACTION_PARAM1 = 6,
    EI_ADDR_ACTION_PARAM2 = 7,
};

//eac执行sros子任务的
struct StSrosActCmd {
 public:
    SrosActControl sros_act_control_ = SAC_NONE;
    uint32_t action_no_ = 0;
    uint16_t action_id_ = 0;
    uint16_t action_param0_ = 0;
    uint16_t action_param1_ = 0;
    uint16_t action_param2_ = 0;
};

class SimplifyModbusActionController {
 public:
    SimplifyModbusActionController(uint16_t basis_input_registers_addr, uint16_t basis_hold_registers_addr,
                                   uint16_t extend_input_registers_addr, uint16_t extend_hold_registers_addr);

    uint16_t getBasisInputRegistersAddr() { return basis_input_registers_addr_; }
    uint16_t getBasisHoldRegistersAddr() { return basis_hold_registers_addr_; }
    uint16_t getExtendInputRegistersAddr() { return extend_input_registers_addr_; }
    uint16_t getExtendHoldRegistersAddr() { return extend_hold_registers_addr_; }

    //基础功能读取参数
    EacSysState getEacSysState(uint16_t &eac_sys_error_code);
    EacActState getEacActState(uint32_t &eac_act_result, uint32_t &run_action_no);
    uint16_t getEacLoadState();
    uint16_t getEacAssistFunc();
    uint16_t getEacHeartbeat();
    std::string getEacVersion();

    bool isBanMoveTask(); // 是否禁止移动

    //基础功能设置参数
    void getEacParamControl(EacSysControl& eac_sys_control, EacActControl& eac_act_control);

    void resetEacSysControl();
    void resetEacActControl();

    void resetEac();
    void clearEac();
    void pauseAction();
    void continueAction();
    void cancelAction();
    void newAction(uint32_t action_no, uint16_t action_id, uint16_t action_param0, uint16_t action_param1);


    //扩展功能读取参数
    SrosSysControl getSrosSysControl();
    void getStSrosActCmd(StSrosActCmd &stSrosActCmd);
    //<

    //扩展功能设置参数
    void setStSrosActRsp(SrosActState sros_act_state, uint32_t sros_act_result, uint32_t run_action_no);
    //<


 private:
    inline std::vector<uint16_t> readBasisInputRegisters(uint16_t address, uint16_t count = 1) {
        return register_admin_->readInputRegisters(basis_input_registers_addr_ + address, count);
    }

    inline uint16_t readBasisHoldRegisterUint16(uint16_t address) {
        return register_admin_->readHoldRegisterUint16(basis_hold_registers_addr_ + address);
    }
    inline uint32_t readBasisHoldRegisterUint32(uint16_t address) {
        return register_admin_->readHoldRegisterUint32(basis_hold_registers_addr_ + address);
    }
    inline std::vector<uint16_t> readBasisHoldRegisters(uint16_t address, uint16_t count = 1){
        return register_admin_->readHoldRegisters(basis_hold_registers_addr_ + address, count);
    }
    inline void writeBasisInputRegisterUint16(uint16_t address, uint16_t value) {
        register_admin_->writeInputRegisterUint16(basis_input_registers_addr_ + address, value);
    }
    inline void writeBasisInputRegisters(uint16_t address, std::vector<uint16_t> &values) {
        register_admin_->writeInputRegisters(basis_input_registers_addr_ + address, values);
    }

    inline uint16_t readExtendHoldRegisterUint16(uint16_t address) {
        return register_admin_->readHoldRegisterUint16(extend_hold_registers_addr_ + address);
    }
    inline uint32_t readExtendHoldRegisterUint32(uint16_t address) {
        return register_admin_->readHoldRegisterUint32(extend_hold_registers_addr_ + address);
    }
    inline std::vector<uint16_t> readExtendHoldRegisters(uint16_t address, uint16_t count = 1){
        return register_admin_->readHoldRegisters(extend_hold_registers_addr_ + address, count);
    }
    inline void writeExtendInputRegisterUint16(uint16_t address, uint16_t value) {
        register_admin_->writeInputRegisterUint16(extend_input_registers_addr_ + address, value);
    }
    inline void writeExtendInputRegisters(uint16_t address, std::vector<uint16_t> &values) {
        register_admin_->writeInputRegisters(extend_input_registers_addr_ + address, values);
    }

    // sros modbus内部寄存器同步的起始地址
    uint16_t basis_input_registers_addr_ = 0;  // 基础输入寄存器起始地址
    uint16_t basis_hold_registers_addr_ = 0;   // 基础保持寄存器起始地址
    uint16_t extend_input_registers_addr_ = 0; // 扩展输入寄存器起始地址
    uint16_t extend_hold_registers_addr_ = 0;  // 扩展保持寄存器起始地址

    core::RegisterAdmin *register_admin_ = core::RegisterAdmin::getInstance();
};
}  // namespace modbus
}  // namespace sros

#endif  // SROS_SIMPLIFY_MODBUS_ACTION_CONTROLLER_H
