//
// Created by caoyan on 6/22/21.
//

#include "modbus_synchronizer.h"
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <thread>
#include "core/logger.h"

namespace sros {
namespace modbus {

ModbusSynchronizer::ModbusSynchronizer() {}

void ModbusSynchronizer::setReplaceInputRegistersWithHoldRegisters(bool replace_input_registers_with_hold) {
    replace_input_registers_with_hold_ = replace_input_registers_with_hold;
}

void ModbusSynchronizer::setEnableExtendFunc(bool enable_extend_func) {
    enable_extend_func_ = enable_extend_func;
}

void ModbusSynchronizer::setBasisInputRegister(uint16_t local_basis_input_register_addr,
                                               uint16_t remote_basis_hold_register_addr,
                                               uint16_t remote_basis_hold_register_count) {
    local_basis_input_register_addr_ = local_basis_input_register_addr;
    remote_basis_hold_register_addr_ = remote_basis_hold_register_addr;
    remote_basis_hold_register_count_ = remote_basis_hold_register_count;
}

void ModbusSynchronizer::setBasisHoldRegister(uint16_t local_basis_hold_register_addr,
                                              uint16_t remote_basis_input_register_addr,
                                              uint16_t remote_basis_input_register_count) {

    local_basis_hold_register_addr_ = local_basis_hold_register_addr;
    remote_basis_input_register_addr_ = remote_basis_input_register_addr;
    remote_basis_input_register_count_ = remote_basis_input_register_count;
}

void ModbusSynchronizer::setExtendInputRegister(uint16_t local_extend_input_register_addr,
                                                uint16_t remote_extend_hold_register_addr,
                                                uint16_t remote_extend_hold_register_count) {

    local_extend_input_register_addr_ = local_extend_input_register_addr;
    remote_extend_hold_register_addr_ = remote_extend_hold_register_addr;
    remote_extend_hold_register_count_ = remote_extend_hold_register_count;
}

void ModbusSynchronizer::setExtendHoldRegister(uint16_t local_extend_hold_register_addr,
                                               uint16_t remote_extend_input_register_addr,
                                               uint16_t remote_extend_input_register_count) {
    local_extend_hold_register_addr_ = local_extend_hold_register_addr;
    remote_extend_input_register_addr_ = remote_extend_input_register_addr;
    remote_extend_input_register_count_ = remote_extend_input_register_count;
}


void ModbusSynchronizer::setRemoteTcpInfo(const std::string &ip, int port) {
    ip_ = ip;
    port_ = port;
    use_backend_ = MODBUS_TCP;
}

void ModbusSynchronizer::setRemoteRtuInfo(const std::string &device_name, int baud_rate) {
    device_name_rtu_ = device_name;
    baud_rate_ = baud_rate;
    use_backend_ = MODBUS_RTU;
}

void ModbusSynchronizer::setSlaveId(int slave_id) {
    if (slave_id > 256 || slave_id < 0) {
        slave_id_ = 1;
        LOGGER(ERROR, DEVICE) << "Modbus Synchronizer Bad slave id " << slave_id << " force set slave id to 1!!!";
    } else {
        slave_id_ = slave_id;
    }
}

void ModbusSynchronizer::run() {
    LOGGER(INFO, DEVICE) << "Modbus Synchronizer run! " << getRemoteInfo() << "\n"
                         << "sync basis (" << remote_basis_hold_register_count_ << " count) input register "
                         << local_basis_input_register_addr_ << " -> " << remote_basis_hold_register_addr_ << "\n"
                         << "sync basis (" << remote_basis_input_register_count_ << " count) hold register "
                         << local_basis_hold_register_addr_ << " -> " << remote_basis_input_register_addr_ << "\n";

    if(enable_extend_func_) {
        LOGGER(INFO, DEVICE) << "sync extend (" << remote_extend_hold_register_count_ << " count) input register "
                             << local_extend_input_register_addr_ << " -> " << remote_extend_hold_register_addr_ << "\n"
                             << "sync extend (" << remote_extend_input_register_count_ << " count) hold register "
                             << local_extend_hold_register_addr_ << " -> " << remote_extend_input_register_addr_;
    }

    state_ = RUN;

    boost::thread(boost::bind(&ModbusSynchronizer::loop, this));
}

std::string ModbusSynchronizer::getRemoteInfo() const {
    std::string remote_info;
    if (use_backend_ == MODBUS_TCP) {
        remote_info = "modbus-TCP " + ip_ + " " + std::to_string(port_);
    } else {
        remote_info = "modbus-RTU " + device_name_rtu_ + " " + std::to_string(baud_rate_);
    }

    remote_info += " " + std::to_string(slave_id_);

    return remote_info;
}

bool ModbusSynchronizer::init() {
    if (ctx_ != NULL) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = NULL;
    }

    modbus_free(ctx_);
    if (use_backend_ == MODBUS_TCP) {
        ctx_ = modbus_new_tcp(ip_.data(), port_);
    } else if (use_backend_ == MODBUS_RTU) {
        //        auto &s = sros::core::Settings::getInstance();
        //        auto parity_str = s.getValue<std::string>("modbus.modbus_rtu_parity", "NoneParity");
        char parity = 'N';
        //        if (parity_str == "OddParity") {
        //            parity = 'O';
        //        } else if (parity_str == "EvenParity") {
        //            parity = 'E';
        //        }
        ctx_ = modbus_new_rtu(device_name_rtu_.c_str(), baud_rate_, parity, 8, 1);
    } else {
        LOGGER(ERROR, MODBUS) << "Modbus Synchronizer UNREACHABLE! unkown backend" << use_backend_;  // UNREACHABLE
    }

    modbus_set_slave(ctx_, slave_id_);

    modbus_set_debug(ctx_, FALSE);

    if (modbus_connect(ctx_) == -1) {
        LOGGER(ERROR, DEVICE) << "Modbus Synchronizer connection failed: " << modbus_strerror(errno);
        modbus_free(ctx_);
        ctx_ = NULL;
        return false;
    }

    LOGGER(INFO, DEVICE) << "Modbus Synchronizer connected!";

    return true;
}
void ModbusSynchronizer::loop() {
    while (true) {
        if (state_ == RUN) {
            if (connected_) {
                synchronize();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                if (init()) {
                    connected_ = true;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                } else {
                    connected_ = false;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}
bool ModbusSynchronizer::synchronize() {
    //同步基础功能的寄存器
    //将sros的basis的输入寄存器同步写入到eac的保持寄存器
    auto values = reg_admin_->readInputRegisters(local_basis_input_register_addr_, remote_basis_hold_register_count_);
    int rc =
        modbus_write_registers(ctx_, remote_basis_hold_register_addr_, remote_basis_hold_register_count_, values.data());
    if (rc == -1) {
        LOGGER(ERROR, DEVICE) << "Modbus Synchronizer Modbus write registers " << errno << " "
                              << modbus_strerror(errno);
        if (errno < MODBUS_ENOBASE) {
            connected_ = false;
            synchronizer_ok_ = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return false;
        }
    }

    //将eac的basis的输入寄存器同步写入到sros的保持寄存器
    values.resize(remote_basis_input_register_count_);
    if (replace_input_registers_with_hold_) {
        rc = modbus_read_registers(ctx_, remote_basis_input_register_addr_, remote_basis_input_register_count_,
                                         values.data());
    }else {
        rc = modbus_read_input_registers(ctx_, remote_basis_input_register_addr_, remote_basis_input_register_count_,
                                         values.data());
    }
    if (rc == -1) {
        LOGGER(ERROR, DEVICE) << "Modbus Synchronizer Modbus read input registers " << errno << " "
                              << modbus_strerror(errno);
        if (errno < MODBUS_ENOBASE) {
            connected_ = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        synchronizer_ok_ = false;
        return false;
    }
    reg_admin_->writeHoldRegisters(local_basis_hold_register_addr_, values);

    //是否开启扩展寄存器的同步
    if (enable_extend_func_) {
        //同步扩展功能的寄存器
        //将sros的extend的输入寄存器同步写入到eac的保持寄存器
        values = reg_admin_->readInputRegisters(local_extend_input_register_addr_, remote_extend_hold_register_count_);
        rc = modbus_write_registers(ctx_, remote_extend_hold_register_addr_, remote_extend_hold_register_count_, values.data());
        if (rc == -1) {
            LOGGER(ERROR, DEVICE) << "Modbus Synchronizer Modbus write registers " << errno << " "
                                << modbus_strerror(errno);
            if (errno < MODBUS_ENOBASE) {
                connected_ = false;
                synchronizer_ok_ = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                return false;
            }
        }

        //将eac的extend的输入寄存器同步写入到sros的保持寄存器
        values.resize(remote_extend_input_register_count_);
        if (replace_input_registers_with_hold_) {
            rc = modbus_read_registers(ctx_, remote_extend_input_register_addr_, remote_extend_input_register_count_,
                                    values.data());
        }else {
            rc = modbus_read_input_registers(ctx_, remote_extend_input_register_addr_, remote_extend_input_register_count_,
                                            values.data());
        }
        if (rc == -1) {
            LOGGER(ERROR, DEVICE) << "Modbus Synchronizer Modbus read input registers " << errno << " "
                                << modbus_strerror(errno);
            if (errno < MODBUS_ENOBASE) {
                connected_ = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            synchronizer_ok_ = false;
            return false;
        }
        reg_admin_->writeHoldRegisters(local_extend_hold_register_addr_, values);
    }

    //返回
    synchronizer_ok_ = true;
    return true;
}
}  // namespace modbus
}  // namespace sros