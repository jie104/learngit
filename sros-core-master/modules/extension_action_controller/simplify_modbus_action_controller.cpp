//
// Created by caoyan on 6/22/21.
//

#include "simplify_modbus_action_controller.h"
#include <glog/logging.h>
#include "core/logger.h"
#include "core/util/utils.h"
#include "thirty-party/libmodbus/modbus.h"

using namespace sros::core;
namespace sros {
namespace eac {

SimplifyModbusActionController::SimplifyModbusActionController(uint16_t basis_input_registers_addr,
                                                               uint16_t basis_hold_registers_addr,
                                                               uint16_t extend_input_registers_addr,
                                                               uint16_t extend_hold_registers_addr) {

    basis_input_registers_addr_ = basis_input_registers_addr;  // 基础输入寄存器起始地址
    basis_hold_registers_addr_ = basis_hold_registers_addr;   // 基础保持寄存器起始地址
    extend_input_registers_addr_ = extend_input_registers_addr; // 扩展输入寄存器起始地址
    extend_hold_registers_addr_ = extend_hold_registers_addr;  // 扩展保持寄存器起始地址
}

EacSysState SimplifyModbusActionController::getEacSysState(uint16_t &eac_sys_error_code) {
    auto values = readBasisHoldRegisters(BI_ADDR_EAC_SYS_STATE, 2);
    auto eac_sys_state = static_cast<EacSysState>(values[0]);
    eac_sys_error_code = values[1];

    return eac_sys_state;
}

EacActState SimplifyModbusActionController::getEacActState(uint32_t &eac_act_result, uint32_t &run_action_no) {
    auto values = readBasisHoldRegisters(BI_ADDR_EAC_ACT_STATE, 5);
    auto eac_act_state = static_cast<EacActState>(values[0]);
    eac_act_result = MODBUS_GET_INT32_FROM_INT16(values.data(), 1);
    run_action_no = MODBUS_GET_INT32_FROM_INT16(values.data(), 3);

    return eac_act_state;
}

std::string SimplifyModbusActionController::getEacVersion() {
    uint16_t version = readBasisHoldRegisterUint16(BI_ADDR_EAC_SYS_VERSION);
    return versionUint16ToStr(version);
}

uint16_t SimplifyModbusActionController::getEacHeartbeat() {
    return readBasisHoldRegisterUint16(BI_ADDR_HEART_BEAT);
}

uint16_t SimplifyModbusActionController::getEacLoadState() {
    return readBasisHoldRegisterUint16(BI_ADDR_LOAD_STATE);
}

uint16_t SimplifyModbusActionController::getEacAssistFunc() {
    return readBasisHoldRegisterUint16(BI_ADDR_ASSIS_FUNC);
}

bool SimplifyModbusActionController::isBanMoveTask() {
    return (getEacAssistFunc() & AF_BAN_MOVE_TASK);
}

void SimplifyModbusActionController::getEacParamControl(EacSysControl& eac_sys_control,
                                                        EacActControl& eac_act_control) {
    auto values = readBasisInputRegisters(BH_ADDR_EAC_SYS_CONTROL, 2);
    //LOG(INFO) << "getEacParamControl: " << values[0] << ", " << values[1];
    eac_sys_control = static_cast<EacSysControl>(values[0]);
    eac_act_control = static_cast<EacActControl>(values[1]);
}

void SimplifyModbusActionController::resetEacSysControl() {
    writeBasisInputRegisterUint16(BH_ADDR_EAC_SYS_CONTROL, ESC_NONE);
}


void SimplifyModbusActionController::resetEac() {
    //writeBasisInputRegisterUint16(BH_ADDR_EAC_SYS_CONTROL, ESC_RESET);
    std::vector<uint16_t> values(7, 0);
    values[0] = ESC_RESET;
    writeBasisInputRegisters(BH_ADDR_EAC_SYS_CONTROL, values);
}

void SimplifyModbusActionController::clearEac() {
    //writeBasisInputRegisterUint16(BH_ADDR_EAC_SYS_CONTROL, ESC_CLEAR);

    std::vector<uint16_t> values(7, 0);
    values[0] = ESC_CLEAR;
    writeBasisInputRegisters(BH_ADDR_EAC_SYS_CONTROL, values);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


void SimplifyModbusActionController::resetEacActControl() {
    writeBasisInputRegisterUint16(BH_ADDR_EAC_ACT_CONTROL, EAC_NONE);
}

void SimplifyModbusActionController::newAction(uint32_t action_no, uint16_t action_id,
                                               uint16_t action_param0, uint16_t action_param1) {

    std::vector<uint16_t> values(6);
    values[0] = EAC_START_ACTION;
    MODBUS_SET_INT32_TO_INT16(values.data(), 1, action_no);
    values[3] = action_id;
    values[4] = action_param0;
    values[5] = action_param1;
    writeBasisInputRegisters(BH_ADDR_EAC_ACT_CONTROL, values);

    setStSrosActRsp(SAS_IDLE, 0x00, 0x00);
}

void SimplifyModbusActionController::pauseAction() {
    writeBasisInputRegisterUint16(BH_ADDR_EAC_ACT_CONTROL, EAC_PAUSE_ACTION);
}

void SimplifyModbusActionController::continueAction() {
    writeBasisInputRegisterUint16(BH_ADDR_EAC_ACT_CONTROL, EAC_CONTINUE_ACTION);
}

void SimplifyModbusActionController::cancelAction() {
    writeBasisInputRegisterUint16(BH_ADDR_EAC_ACT_CONTROL, EAC_CANCEL_ACTION);
}

//extend
SrosSysControl SimplifyModbusActionController::getSrosSysControl() {
    auto sros_sys_control = readExtendHoldRegisterUint16(EI_ADDR_SROS_SYS_CONTROL);
    return static_cast<SrosSysControl>(sros_sys_control);
}


void SimplifyModbusActionController::getStSrosActCmd(StSrosActCmd &stSrosActCmd) {
    auto values = readExtendHoldRegisters(EI_ADDR_SROS_ACT_CONTROL, 7);
    stSrosActCmd.sros_act_control_ = static_cast<SrosActControl>(values[0]);
    stSrosActCmd.action_no_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 1);
    stSrosActCmd.action_id_ = values[3];
    stSrosActCmd.action_param0_ = values[4];
    stSrosActCmd.action_param1_ = values[5];
    stSrosActCmd.action_param2_ = values[6];
}

void SimplifyModbusActionController::setStSrosActRsp(SrosActState sros_act_state,
                                                     uint32_t sros_act_result, uint32_t run_action_no) {
    std::vector<uint16_t> values(5);
    values[0] = (uint16_t)sros_act_state;
    MODBUS_SET_INT32_TO_INT16(values.data(), 1, sros_act_result);
    MODBUS_SET_INT32_TO_INT16(values.data(), 3, run_action_no);
    writeExtendInputRegisters(EH_ADDR_SROS_ACT_STATE, values);
}

}  // namespace modbus
}  // namespace sros