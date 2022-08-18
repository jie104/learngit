//
// Created by huangwuxian on 19-3-2.
//

#include "register_admin.h"
#include "core/settings.h"

namespace sros {
namespace core {

RegisterAdmin::RegisterAdmin() {
    auto &s = sros::core::Settings::getInstance();
    auto start_bits_addr = s.getValue<int>("modbus.start_bits_addr", 00001);
    auto start_input_bits_addr = s.getValue<int>("modbus.start_input_bits_addr", 10001);
    auto start_input_registers_addr = s.getValue<int>("modbus.start_input_registers_addr", 30001);
    auto start_registers_addr = s.getValue<int>("modbus.start_registers_addr", 40001);
    mb_mapping_ = modbus_mapping_new_start_address(start_bits_addr, 0x64, start_input_bits_addr, 0x64,
                                                   start_registers_addr, 10000, start_input_registers_addr, 10000);
}

RegisterAdmin::~RegisterAdmin() {
    if (mb_mapping_ != NULL) {
        modbus_mapping_free(mb_mapping_);
    }
}

RegisterAdmin *RegisterAdmin::getInstance() {
    static RegisterAdmin instance;
    return &instance;
}

void RegisterAdmin::setDiscreteInput(uint16_t addr, uint16_t value, bool lock) {
    if (addr < 0 || addr > nbInputBits()) {
        LOG(ERROR) << "RegisterAdmin::setDiscreteInput(): Invalid input discrete address: " << addr;
        return;
    }

    if (lock) {
        unique_lock locker(mutex_);
        mb_mapping_->tab_input_bits[addr] = value;
    } else {
        mb_mapping_->tab_input_bits[addr] = value;
    }
}

void RegisterAdmin::setInputRegister16(uint16_t addr, uint16_t value, bool lock) {
    if (addr < 0 || addr > nbInputRegisters()) {
        LOG(ERROR) << "RegisterAdmin::setInputRegister16(): Invalid input register address: " << addr;
        return;
    }

    if (lock) {
        unique_lock locker(mutex_);
        mb_mapping_->tab_input_registers[addr] = value;
    } else {
        mb_mapping_->tab_input_registers[addr] = value;
    }
}

void RegisterAdmin::setInputRegister32(uint16_t addr, uint32_t value, bool lock) {
    if (addr < 0 || addr > nbInputRegisters()) {
        LOG(ERROR) << "RegisterAdmin::setInputRegister32(): Invalid input register address: " << addr;
        return;
    }

    if (lock) {
        unique_lock locker(mutex_);
        MODBUS_SET_INT32_TO_INT16(mb_mapping_->tab_input_registers, addr, value);
    } else {
        MODBUS_SET_INT32_TO_INT16(mb_mapping_->tab_input_registers, addr, value);
    }
}

void RegisterAdmin::setInputRegister32(uint16_t addr, int32_t value, bool lock) {
    if (addr < 0 || addr > nbInputRegisters()) {
        LOG(ERROR) << "RegisterAdmin::setInputRegister32(): Invalid input register address: " << addr;
        return;
    }

    if (lock) {
        unique_lock locker(mutex_);
        MODBUS_SET_INT32_TO_INT16(mb_mapping_->tab_input_registers, addr, value);
    } else {
        MODBUS_SET_INT32_TO_INT16(mb_mapping_->tab_input_registers, addr, value);
    }
}

bool RegisterAdmin::doReadRegister(uint16_t addr, uint32_t &value, ModbusAddrType eType) {
    unique_lock locker(mutex_);
    uint16_t mapping_addr = 0;
    switch (eType) {
        case ModbusAddrType::AddrDiscreteInput: {
            mapping_addr = mapAddrInputBits(addr);
            value = mb_mapping_->tab_input_bits[mapping_addr];
            break;
        }
        case ModbusAddrType::AddrCoil: {
            mapping_addr = mapAddrBits(addr);
            value = mb_mapping_->tab_bits[mapping_addr];
            break;
        }
        case ModbusAddrType::AddrInputRegister: {
            mapping_addr = mapAddrInputRegisters(addr);
            switch (mapping_addr) {
                // TODO 带符号类型的暂不支持
                case IRA_POSE:
                case IRA_SPEED_FOR_X_STATE:
                case IRA_SPEED_FOR_Y_STATE:
                case IRA_SPEED_FOR_YAW_STATE: {
                    LOG(ERROR) << "RegisterAdmin::doWriteRegister(): Unsupport read register address: " << addr;
                    return false;
                }
                case IRA_HARDWARE_ERROR_CODE:
                case IRA_SYSTEM_LAST_ERROR:
                case IRA_TOTAL_MILEAGE:
                case IRA_POWERON_TIME:
                case IRA_POWER_CYCLE:
                case IRA_SYSTEM_TIME:
                case IRA_ACTION_RESULT_ABANDONED: {
                    value = MODBUS_GET_INT32_FROM_INT16(mb_mapping_->tab_input_registers, mapping_addr);
                    break;
                }
                default: {
                    value = mb_mapping_->tab_input_registers[mapping_addr];
                    break;
                }
            }
            break;
        }
        case ModbusAddrType::AddrHoldingRegister: {
            mapping_addr = mapAddrHoldingRegisters(addr);
            switch (mapping_addr) {
                // TODO 带符号类型的暂不支持
                case HRA_START_LOCATION_POSE:
                case HRA_MOVE_TO_POSE:
                case HRA_MANUAL_CONTROL_V_X:
                case HRA_MANUAL_CONTROL_V_Y:
                case HRA_MANUAL_CONTROL_V_YAW: {
                    LOG(ERROR) << "RegisterAdmin::doWriteRegister(): Unsupport write register address: " << addr;
                    return false;
                }
                default: {
                    value = mb_mapping_->tab_registers[mapping_addr];
                    break;
                }
            }
            break;
        }
        default: {
            LOG(ERROR) << "RegisterAdmin::doWriteRegister(): Invalid register address: " << addr;
            return false;
        }
    }

    return true;
}

bool RegisterAdmin::doWriteRegister(uint16_t addr, uint32_t value, ModbusAddrType eType) {
    LOG(INFO) << "RegisterAdmin::doWriteRegister(): " << addr << " " << value;

    unique_lock locker(mutex_);
    uint16_t mapping_addr = 0;
    switch (eType) {
        case ModbusAddrType::AddrCoil: {
            mapping_addr = mapAddrBits(addr);
            if (value == 0xFF00 || value == 0x0) {
                mb_mapping_->tab_bits[mapping_addr] = value ? ON : OFF;
            }
            break;
        }
        case ModbusAddrType::AddrHoldingRegister: {
            mapping_addr = mapAddrHoldingRegisters(addr);
            switch (mapping_addr) {
                // TODO 带符号类型的暂不支持
                case HRA_START_LOCATION_POSE:
                case HRA_MOVE_TO_POSE:
                case HRA_MANUAL_CONTROL_V_X:
                case HRA_MANUAL_CONTROL_V_Y:
                case HRA_MANUAL_CONTROL_V_YAW: {
                    LOG(ERROR) << "RegisterAdmin::doWriteRegister(): Unsupport write register address: " << addr;
                    return false;
                }
                default: {
                    mb_mapping_->tab_registers[mapping_addr] = (uint16_t)value;
                    break;
                }
            }
            break;
        }
        case ModbusAddrType::AddrDiscreteInput:
        case ModbusAddrType::AddrInputRegister: {
            // 只读寄存器
            break;
        }
        default: {
            LOG(ERROR) << "RegisterAdmin::doWriteRegister(): Invalid register address: " << addr;
            return false;
        }
    }
    return true;
}

ModbusAddrType RegisterAdmin::getRegisterAddrType(uint16_t addr) const {
    ModbusAddrType addr_type = ModbusAddrType::AddrNone;
    if (addr >= startInputBits() && addr < (startInputBits() + nbInputBits())) {
        addr_type = ModbusAddrType::AddrDiscreteInput;
    } else if (addr >= startBits() && addr < (startBits() + nbBits())) {
        addr_type = ModbusAddrType::AddrCoil;
    } else if (addr >= startInputRegisters() && addr < (startInputRegisters() + nbInputRegisters())) {
        addr_type = ModbusAddrType::AddrInputRegister;
    } else if (addr >= startHoldingRegisters() && addr < (startHoldingRegisters() + nbHoldingRegisters())) {
        addr_type = ModbusAddrType::AddrHoldingRegister;
    } else {
        //        LOG(ERROR) << "RegisterAdmin::getAddrType(): Invalid register address: " << addr;
    }

    return addr_type;
}

bool RegisterAdmin::writeCoilRegister(uint16_t addr, uint32_t value) {
    return doWriteRegister(addr, value, ModbusAddrType::AddrCoil);
}

bool RegisterAdmin::writeHoldingRegister(uint16_t addr, uint32_t value) {
    return doWriteRegister(addr, value, ModbusAddrType::AddrHoldingRegister);
}

uint16_t RegisterAdmin::absoluteAddr(uint16_t addr, ModbusAddrType eType) const {
    uint16_t register_addr = addr;
    switch (eType) {
        case ModbusAddrType::AddrCoil: {
            register_addr = absoluteAddrBits(addr);
            break;
        }
        case ModbusAddrType::AddrDiscreteInput: {
            register_addr = absoluteAddrInputBits(addr);
            break;
        }
        case ModbusAddrType::AddrInputRegister: {
            register_addr = absoluteAddrInputRegisters(addr);
            break;
        }
        case ModbusAddrType::AddrHoldingRegister: {
            register_addr = absoluteAddrHoldingRegisters(addr);
            break;
        }
        default: {
            break;
        }
    }
    return register_addr;
}

std::vector<uint16_t> RegisterAdmin::readInputRegisters(uint16_t address, uint16_t count) {
    std::vector<uint16_t> values(count);

    auto mapping_addr = mapAddrInputRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto i = 0; i < count; ++i) {
        values[i] = mb_mapping_->tab_input_registers[mapping_addr + i];
    }

    return values;
}

void RegisterAdmin::writeInputRegisters(uint16_t address, std::vector<uint16_t> &values) {
    auto mapping_addr = mapAddrInputRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto i = 0; i < values.size(); ++i) {
        mb_mapping_->tab_input_registers[mapping_addr + i] = values[i];
    }
}

void RegisterAdmin::writeRelativeInputRegisters(uint16_t address, std::vector<uint16_t> &values) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto i = 0; i < values.size(); ++i) {
        mb_mapping_->tab_input_registers[address + i] = values[i];
    }
}

std::vector<uint16_t> RegisterAdmin::readHoldRegisters(uint16_t address, uint16_t count) {
    std::vector<uint16_t> values(count);

    auto mapping_addr = mapAddrHoldingRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto i = 0; i < count; ++i) {
        values[i] = mb_mapping_->tab_registers[mapping_addr + i];
    }

    return values;
}

void RegisterAdmin::writeHoldRegisters(uint16_t address, std::vector<uint16_t> &values) {
    auto mapping_addr = mapAddrHoldingRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto i = 0; i < values.size(); ++i) {
        mb_mapping_->tab_registers[mapping_addr + i] = values[i];
    }
}

uint16_t RegisterAdmin::readInputRegisterUint16(uint16_t address) {
    auto mapping_addr = mapAddrInputRegisters(address);

    return mb_mapping_->tab_input_registers[mapping_addr];
}

uint16_t RegisterAdmin::readHoldRegisterUint16(uint16_t address) {
    auto mapping_addr = mapAddrHoldingRegisters(address);

    return mb_mapping_->tab_registers[mapping_addr];
}

uint32_t RegisterAdmin::readInputRegisterUint32(uint16_t address) {
    auto mapping_addr = mapAddrInputRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    return MODBUS_GET_INT32_FROM_INT16(mb_mapping_->tab_input_registers, mapping_addr);
}

uint32_t RegisterAdmin::readHoldRegisterUint32(uint16_t address) {
    auto mapping_addr = mapAddrHoldingRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    return MODBUS_GET_INT32_FROM_INT16(mb_mapping_->tab_registers, mapping_addr);
}

void RegisterAdmin::writeInputRegisterUint16(uint16_t address, uint16_t value) {
    auto mapping_addr = mapAddrInputRegisters(address);

    // 一次写一个寄存器可以不需要加锁
    mb_mapping_->tab_input_registers[mapping_addr] = value;
}

void RegisterAdmin::writeHoldRegisterUint16(uint16_t address, uint16_t value) {
    auto mapping_addr = mapAddrHoldingRegisters(address);

    mb_mapping_->tab_registers[mapping_addr] = value;
}

void RegisterAdmin::writeInputRegisterUint32(uint16_t address, uint32_t value) {
    auto mapping_addr = mapAddrInputRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    MODBUS_SET_INT32_TO_INT16(mb_mapping_->tab_input_registers, mapping_addr, value);
}

void RegisterAdmin::writeHoldRegisterUint32(uint16_t address, uint32_t value) {
    auto mapping_addr = mapAddrHoldingRegisters(address);

    std::lock_guard<std::mutex> lock(mutex_);
    MODBUS_SET_INT32_TO_INT16(mb_mapping_->tab_registers, mapping_addr, value);
}

}  // namespace core
}  // namespace sros
