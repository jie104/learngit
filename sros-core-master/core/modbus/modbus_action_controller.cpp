/**
 * @file modbus_action_controller
 *
 * @author pengjiali
 * @date 19-11-13.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "modbus_action_controller.h"
#include <glog/logging.h>
#include "core/logger.h"
#include "core/util/utils.h"
#include "thirty-party/libmodbus/modbus.h"

using namespace sros::core;
namespace sros {
namespace modbus {

// WAIT_RESPONSE_INTERVAL_TIME_MS * WAIT_RESPONSE_RETRY_TIMES = 2s
const int WAIT_RESPONSE_INTERVAL_TIME_MS = 50;  // 轮训等待结果的间隔时间
const int WAIT_RESPONSE_RETRY_TIMES = 40;       // 轮训等待结果超时次数

ModbusActionController::ModbusActionController(int start_input_registers_addr, int start_hold_registers_addr)
    : start_input_registers_addr_(start_input_registers_addr), start_hold_registers_addr_(start_hold_registers_addr) {
    writeInputRegisterUint16(ACTION_DRIVE_STATE, DRIVE_STATE_RUN);
}

std::string ModbusActionController::getVersion() {
    uint32_t version = readHoldRegisterUint32(ACTION_CONTROLLER_VERSION);
    return versionUint2Str(version);
}

int ModbusActionController::getType() { return readHoldRegisterUint16(ACTION_CONTROLLER_TYPE); }

int ModbusActionController::getHeartbeat() { return readHoldRegisterUint32(HEART_BEAT); }

uint16_t ModbusActionController::getMultiLoadState() { return readHoldRegisterUint16(MULTI_LOAD_STATE); }

SystemState ModbusActionController::getSystemState(int &error_no) {
    auto values = readHoldRegisters(SYSTEM_STATE, 3);
    auto system_state = static_cast<SystemState>(values[0]);
    error_no = MODBUS_GET_INT32_FROM_INT16(values.data(), 1);

    //    if (system_state == SYSTEM_STATE_IDLE) {
    //        clear();
    //    }

    return system_state;
}

bool ModbusActionController::reset() {
    LOGGER(INFO, DEVICE) << "Reset EAC";
    writeInputRegisterUint16(RESET, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    writeInputRegisterUint16(RESET, 0);
    return true;
}

void ModbusActionController::getActionStatus(ActionStatus &action_status) {
    auto values = readHoldRegisters(ACTION_STATE, 16);
    action_status.action_state_ = static_cast<TaskState>(values[0]);
    action_status.action_no_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 5);
    action_status.action_id_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 7);
    action_status.action_param_0_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 9);
    action_status.action_param_1_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 11);
    action_status.action_result_ = static_cast<TaskResult>(values[13]);
    action_status.action_result_value_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 14);
    action_status.str_reserved = getDebugRegistersStr();
}

void ModbusActionController::getSrosActionStatus(SrosActionState &status) {
    auto values = readHoldRegisters(SROS_ACTION_CONTROL, 9);
    status.sros_action_control_ = static_cast<SrosActionControl>(values[0]);
    status.sros_action_id_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 1);
    status.sros_action_param_0_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 3);
    status.sros_action_param_1_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 5);
    status.sros_action_param_2_ = MODBUS_GET_INT32_FROM_INT16(values.data(), 7);
}

bool ModbusActionController::newAction(int action_no, int action_id, int param1, int param2, NewActionResult &result) {
    LOGGER(INFO, DEVICE) << "Send new action to EAC";

    // 清空上一次新任务结果
    if (!clear()) {
        return false;
    }

    std::vector<uint16_t> values(9);
    values[0] = 0x01;
    MODBUS_SET_INT32_TO_INT16(values.data(), 1, action_no);
    MODBUS_SET_INT32_TO_INT16(values.data(), 3, action_id);
    MODBUS_SET_INT32_TO_INT16(values.data(), 5, param1);
    MODBUS_SET_INT32_TO_INT16(values.data(), 7, param2);
    writeInputRegisters(ACTION_SEQ, values);

    bool ret = false;
    for (auto i = 0; i < WAIT_RESPONSE_RETRY_TIMES; ++i) {
        SystemState systemstate = (SystemState)readHoldRegisterUint16(SYSTEM_STATE);
        result = (NewActionResult)readHoldRegisterUint16(NEW_ACTION_RESULT);
        LOGGER(INFO, DEVICE) << "EAC system state is " << systemstate << ", new action result is " << result;
        if (systemstate == SYSTEM_STATE_RUNNING || result == NAR_SUCCEED) {
            LOGGER(INFO, DEVICE) << "EAC start new action succeed!";
            ret = true;
            break;
        }

        if (result != NAR_SUCCEED && result != NAR_NONE) {
            LOGGER(ERROR, DEVICE) << "EAC start new action failed!!!";
            ret = false;
            break;
        }

        if (i == 39) {
            LOGGER(ERROR, DEVICE) << "EAC start new action failed!!!";
            ret = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_RESPONSE_INTERVAL_TIME_MS));
    }

    writeInputRegisterUint16(ACTION_SEQ, 0);
    return ret;
}

void ModbusActionController::setEmergency(bool enable) {
    LOGGER(INFO, DEVICE) << "Set EAC emergency " << std::boolalpha << enable;
    writeInputRegisterUint16(EMERGENCY, enable ? 1 : 0);
}

bool ModbusActionController::cancelAction() {
    LOGGER(INFO, DEVICE) << "Cancle EAC action";
    bool ret = false;
    writeInputRegisterUint16(CANCEL_ACTION, 1);
    for (auto i = 0; i < WAIT_RESPONSE_RETRY_TIMES; ++i) {
        SystemState systemstate = (SystemState)readHoldRegisterUint16(SYSTEM_STATE);
        if (systemstate != SYSTEM_STATE_RUNNING) {
            ret = true;
            break;
        }

        if (i == 39) {
            LOGGER(ERROR, DEVICE) << "Cancle EAC action failed!!!";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_RESPONSE_INTERVAL_TIME_MS));
    }

    writeInputRegisterUint16(CANCEL_ACTION, 0);
    return ret;
}
bool ModbusActionController::clear() {
    bool ret = true;
    if (readHoldRegisterUint16(NEW_ACTION_RESULT) != 0) {
        LOGGER(INFO, DEVICE) << "Clean EAC";
        writeInputRegisterUint16(CLEAR, 1);
        for (auto i = 0; i < WAIT_RESPONSE_RETRY_TIMES; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_RESPONSE_INTERVAL_TIME_MS));

            auto result = (NewActionResult)readHoldRegisterUint16(NEW_ACTION_RESULT);
            LOGGER(INFO, DEVICE) << "EAC new action result is " << result;

            if (result != NAR_NONE) {
                if (i == WAIT_RESPONSE_RETRY_TIMES - 1) {
                    LOGGER(ERROR, DEVICE) << "EAC clean failed!!!";
                    ret = false;
                }
            } else {
                break;
            }
        }
        writeInputRegisterUint16(CLEAR, 0);
    }

    return ret;
}

bool ModbusActionController::pauseAction() {
    bool ret = true;
    if (readHoldRegisterUint16(ACTION_STATE) != TASK_PAUSED) {
        LOGGER(INFO, DEVICE) << "Send pause to EAC";
        writeInputRegisterUint16(PAUSE, 1);
        for (auto i = 0; i < WAIT_RESPONSE_RETRY_TIMES; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_RESPONSE_INTERVAL_TIME_MS));

            if (readHoldRegisterUint16(ACTION_STATE) != TASK_PAUSED) {
                if (i == WAIT_RESPONSE_RETRY_TIMES - 1) {
                    LOGGER(ERROR, DEVICE) << "EAC pause failed!!!";
                    ret = false;
                }
            } else {
                break;
            }
        }
        writeInputRegisterUint16(PAUSE, 0);
    }

    return ret;
}
bool ModbusActionController::continueAction() {
    bool ret = true;
    if (readHoldRegisterUint16(ACTION_STATE) != TASK_RUNNING) {
        LOGGER(INFO, DEVICE) << "Send continue to EAC";
        writeInputRegisterUint16(CONTINUE, 1);
        for (auto i = 0; i < WAIT_RESPONSE_RETRY_TIMES; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_RESPONSE_INTERVAL_TIME_MS));

            if (readHoldRegisterUint16(ACTION_STATE) != TASK_RUNNING) {
                if (i == WAIT_RESPONSE_RETRY_TIMES - 1) {
                    LOGGER(ERROR, DEVICE) << "EAC continue failed!!!";
                    ret = false;
                }
            } else {
                break;
            }
        }
        writeInputRegisterUint16(CONTINUE, 0);
    }

    return ret;
}

void ModbusActionController::cleanSrosAction() {
    std::vector<uint16_t> data(3, 0);
    writeInputRegisters(SROS_ACTION_STATE, data);
}

void ModbusActionController::updateSrosActionStatus(sros::core::TaskState state, sros::core::TaskResult result,
                                                    int result_value) {
    std::vector<uint16_t> data = {(uint16_t)state, (uint16_t)result, 0, 0};
    MODBUS_SET_INT32_TO_INT16(data.data(), 2, result_value);
    writeInputRegisters(SROS_ACTION_STATE, data);
}

std::string ModbusActionController::getDebugRegistersStr() {
    auto values = readHoldRegisters(DEBUG_REGISTER, 10);
    std::string debug_register_str;
    for(int i = 0; i < 10; i++) {
        int debug_register_val = values[i];
        if( i == 9) {
            std::string temp = std::to_string(debug_register_val);
            int len = temp.length();
            if(len == 1) {
                debug_register_str += "00";
                debug_register_str += temp;
            } else if(len == 2) {
                debug_register_str += "0";
                debug_register_str += temp;
            } else {
                debug_register_str += temp;
            }
        }
        else {
            std::string temp = std::to_string(debug_register_val);
            int len = temp.length();
            if(len == 1) {
                debug_register_str += "00";
                debug_register_str += temp;
            } else if(len == 2) {
                debug_register_str += "0";
                debug_register_str += temp;
            } else {
                debug_register_str += temp;
            }
            debug_register_str += "|";
        }
    }
    return debug_register_str;
}

bool ModbusActionController::isMovementFrohibition() {
    return readHoldRegisterUint16(MOVEMENT_PROHIBITION);
}

}  // namespace modbus
}  // namespace sros