/**
 * @file modbus_action_controller
 *
 * @author pengjiali
 * @date 19-11-13.
 *
 * @describe 动作modbus动作控制器
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MODBUS_ACTION_CONTROLLER_H
#define SROS_MODBUS_ACTION_CONTROLLER_H

#include <string>
#include "core/modbus/register_admin.h"
#include "core/task/task.h"

namespace sros {
namespace modbus {

enum NewActionResult {
    NAR_NONE = 0x00,
    NAR_SUCCEED = 0x01,
    NAR_SYSTEM_BUSY = 0x02,
    NAR_ACTION_ID_NOT_SUPPORT = 0x03,
    NAR_ACTION_PARAM_0_NOT_SUPPORT = 0x04,
    NAR_ACTION_PARAM_1_NOT_SUPPORT = 0x05,
};

enum SystemState {
    SYSTEM_STATE_NONE = 0x00,
    SYSTEM_STATE_INIT = 0x01,
    SYSTEM_STATE_IDLE = 0x02,
    SYSTEM_STATE_RUNNING = 0x03,
    SYSTEM_STATE_ERROR = 0x04,
};

enum DriveState {
    DRIVE_STATE_STOP = 1,
    DRIVE_STATE_RUN = 2,
};

enum SrosActionControl {
    SROS_ACTION_CONTROL_NONE = 0,
    SROS_ACTION_CONTROL_CLEAN = 1,
    SROS_ACTION_CONTROL_START = 2,
    SROS_ACTION_CONTROL_PAUSE = 3,
    SROS_ACTION_CONTROL_CONTINUE = 4,
    SROS_ACTION_CONTROL_CANCEL = 5,
};

enum ActionInputRegisterAddr {
    ACTION_DRIVE_STATE = 0,
    RESET = 1,
    CLEAR = 2,
    CANCEL_ACTION = 3,
    EMERGENCY = 4,
    PAUSE = 5,
    CONTINUE = 6,

    ACTION_SEQ = 15,
    ACTION_NO = 16,
    ACTION_ID = 18,
    ACTION_PARAM_0 = 20,
    ACTION_PARAM_1 = 22,

    SROS_ACTION_STATE = 40,
    SROS_ACTION_RESULT = 41,
    SROS_ACTION_RESULT_VALUE = 42
};

enum ActionHoldRegisterAddr {
    SYSTEM_STATE = 0,
    SYSTEM_ERROR_CODE = 1,
    ACTION_CONTROLLER_VERSION = 3,
    HEART_BEAT = 5,
    ACTION_CONTROLLER_TYPE = 7,
    MULTI_LOAD_STATE = 8,

    NEW_ACTION_RESULT = 10,
    ACTION_STATE = 11,
    MOVEMENT_PROHIBITION = 12, // 禁止移动

    STATE_ACTION_SEQ = 15,
    STATE_ACTION_NO = 16,
    STATE_ACTION_ID = 18,
    STATE_ACTION_PARAM_0 = 20,
    STATE_ACTION_PARAM_1 = 22,
    ACTION_RESULT = 24,
    ACTION_RESULT_VALUE = 25,

    DEBUG_REGISTER = 30,

    SROS_ACTION_CONTROL = 40,
    SROS_ACTION_ID = 41,
    SROS_ACTION_PARAM_0 = 43,
    SROS_ACTION_PARAM_1 = 45,
    SROS_ACTION_PARAM_2 = 47,
};

/**
 * sros 子任务状态
 */
class SrosActionState {
 public:
    SrosActionControl sros_action_control_ = SROS_ACTION_CONTROL_NONE;
    int sros_action_id_ = 0;
    int sros_action_param_0_ = 0;
    int sros_action_param_1_ = 0;
    int sros_action_param_2_ = 0;
};

class ActionStatus {
 public:
    int action_no_ = 0;  // 任务编号
    int action_id_ = 0;  // 动作id
    int action_param_0_ = 0;
    int action_param_1_ = 0;
    sros::core::TaskState action_state_ = core::TASK_NA;           // 动作的状态
    sros::core::TaskResult action_result_ = core::TASK_RESULT_NA;  // 动作的结果
    int action_result_value_ = 0;                                  // 动作执行结果
    std::string str_reserved;   //保留字段
};

class ModbusActionController {
 public:
    ModbusActionController(int start_input_registers_addr, int start_hold_registers_addr);

    std::string getVersion();

    int getType();

    int getHeartbeat();

    uint16_t getMultiLoadState();

    SystemState getSystemState(int &error_no);

    bool reset();

    bool clear();

    void getActionStatus(ActionStatus &action_status);

    void getSrosActionStatus(SrosActionState &status);

    void updateSrosActionStatus(sros::core::TaskState state, sros::core::TaskResult result, int result_value);

    void cleanSrosAction();

    void setEmergency(bool enable);

    bool isMovementFrohibition(); // 是否禁止移动

    bool pauseAction();
    bool continueAction();

    bool newAction(int action_no, int action_id, int param1, int param2, NewActionResult &result);

    bool cancelAction();

    std::string getDebugRegistersStr();

 private:
    // 将漂移地址提取出来，实现部分直接用偏移地址就好了，缩短了代码的长度
    inline uint16_t readHoldRegisterUint16(uint16_t address) {
        return register_admin_->readHoldRegisterUint16(start_hold_registers_addr_ + address);
    }
    inline uint32_t readHoldRegisterUint32(uint16_t address) {
        return register_admin_->readHoldRegisterUint32(start_hold_registers_addr_ + address);
    }
    inline std::vector<uint16_t> readHoldRegisters(uint16_t address, uint16_t count = 1){
        return register_admin_->readHoldRegisters(start_hold_registers_addr_ + address, count);
    }
    inline void writeInputRegisterUint16(uint16_t address, uint16_t value) {
        register_admin_->writeInputRegisterUint16(start_input_registers_addr_ + address, value);
    }
    inline void writeInputRegisters(uint16_t address, std::vector<uint16_t> &values) {
        register_admin_->writeInputRegisters(start_input_registers_addr_ + address, values);
    }

    // 相对于sros来说的
    uint16_t start_input_registers_addr_ = 0;  // 输入寄存器起始地址
    uint16_t start_hold_registers_addr_ = 0;   // 保持寄存器起始地址

    core::RegisterAdmin *register_admin_ = core::RegisterAdmin::getInstance();

    uint16_t seq_ = 0;
};
}  // namespace modbus
}  // namespace sros

#endif  // SROS_MODBUS_ACTION_CONTROLLER_H
