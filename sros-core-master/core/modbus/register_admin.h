//
// Created by huangwuxian on 19-3-2.
//

#ifndef SROS_REGISTER_ADMIN_H
#define SROS_REGISTER_ADMIN_H

#include <mutex>
#include <vector>

#include <thirty-party/libmodbus/modbus.h>
#include "core/modbus/sr_modbus_address.h"

namespace sros {
namespace core {

enum class ModbusAddrType {
    AddrNone = 0,
    AddrCoil,             // 写线圈(读写)
    AddrDiscreteInput,    // 离散输入寄存器(只读)
    AddrHoldingRegister,  // 保存寄存器(读写)
    AddrInputRegister,    // 输入寄存器(只读)
};

typedef std::unique_lock<std::mutex> unique_lock;

class RegisterAdmin {
 public:
    ~RegisterAdmin();

    static RegisterAdmin *getInstance();

    inline modbus_mapping_t *addrMapHandler() { return mb_mapping_; }

    // 线圈(读写)
    inline int nbBits() const { return mb_mapping_->nb_bits; }
    inline int mapAddrBits(uint16_t addr) const {
        return addr - startBits();
    }  // 输入绝对地址，返回相对地址(绝对地址-基地址)
    inline uint16_t absoluteAddrBits(uint16_t addr) const { return addr + startBits(); }  // 根据相对地址返回绝对地址

    // 离散输入寄存器（只读）
    inline int nbInputBits() const { return mb_mapping_->nb_input_bits; }
    inline int mapAddrInputBits(uint16_t addr) const { return addr - startInputBits(); }  // 输入绝对地址，返回相对地址
    inline uint16_t absoluteAddrInputBits(uint16_t addr) const {
        return addr + startInputBits();
    }  // 根据相对地址返回绝对地址

    // 输入寄存器（只读）
    inline int nbInputRegisters() const { return mb_mapping_->nb_input_registers; }
    inline int mapAddrInputRegisters(uint16_t addr) const {
        return addr - startInputRegisters();
    }  // 输入绝对地址，返回相对地址
    inline uint16_t absoluteAddrInputRegisters(uint16_t addr) const {
        return addr + startInputRegisters();
    }  // 根据相对地址返回绝对地址

    // 保持寄存器（读写）
    inline int nbHoldingRegisters() const { return mb_mapping_->nb_registers; }
    inline int mapAddrHoldingRegisters(uint16_t addr) const {
        return addr - startHoldingRegisters();
    }  // 输入绝对地址，返回相对地址
    inline uint16_t absoluteAddrHoldingRegisters(uint16_t addr) const {
        return addr + startHoldingRegisters();
    }  // 根据相对地址返回绝对地址

    uint16_t absoluteAddr(uint16_t addr, ModbusAddrType eType) const;

    // !!!!!!!!!!!!!根据寄存器地址获取寄存器类型
    // 由于modbus寄存器地址不唯一,不同类型的寄存器
    // 地址可能相同，该方法并不可靠，只有在为了兼容
    // Matrix1.7.0及之前版本才调用该方法
    ModbusAddrType getRegisterAddrType(uint16_t addr) const;  // 已废弃

    // 注意输入的地址都是偏移地址（减去基地址）
    void setDiscreteInput(uint16_t addr, uint16_t value, bool lock = true);

    // 设置离散输入寄存器的值
    // 注意输入的地址都是偏移地址（减去基地址）
    // NOTE： int16转为uint16再转为int16符号不会被丢失！
    // 例如：电流可能小于0，单位为int16，modbus存储的寄存器为uint16，
    // 当主机读取电流对应的地址时转换成了int16，该值的符号并没有丢失
    void setInputRegister16(uint16_t addr, uint16_t value, bool lock = true);
    void setInputRegister32(uint16_t addr, uint32_t value, bool lock = true);
    void setInputRegister32(uint16_t addr, int32_t value, bool lock = true);

    // 读写寄存器，addr为绝对地址，函数自动根据地址范围确定寄存器类型
    // TODO 暂不支持负值类型的寄存器
    bool writeCoilRegister(uint16_t addr, uint32_t value);
    bool writeHoldingRegister(uint16_t addr, uint32_t value);
    bool doWriteRegister(uint16_t addr, uint32_t value, ModbusAddrType eType);
    bool doReadRegister(uint16_t addr, uint32_t &value, ModbusAddrType eType);

    // 一些便利的接口，所有读写都是原子操作
    uint16_t readInputRegisterUint16(uint16_t address); // int16也可以用此接口
    uint16_t readHoldRegisterUint16(uint16_t address); // int16也可以用此接口
    uint32_t readInputRegisterUint32(uint16_t address); // int32也可以用此接口
    uint32_t readHoldRegisterUint32(uint16_t address); // int32也可以用此接口
    void writeInputRegisterUint16(uint16_t address, uint16_t value); // int16也可以用此接口
    void writeHoldRegisterUint16(uint16_t address, uint16_t value); // int16也可以用此接口
    void writeInputRegisterUint32(uint16_t address, uint32_t value); // int32也可以用此接口
    void writeHoldRegisterUint32(uint16_t address, uint32_t value); // int32也可以用此接口

    /**
     * 读取输入寄存器数据，读取操作是原子操作
     * @param address 起始地址 绝对地址
     * @param count 读取个数
     * @return 寄存器值的拷贝
     */
    std::vector<uint16_t> readInputRegisters(uint16_t address, uint16_t count = 1);

    /**
     * 写输入寄存器，由于这个是直接写内存所以支持写操作，原子操作
     * @param address 起始地址 绝对地址
     * @param values 需要写入的值
     */
    void writeInputRegisters(uint16_t address, std::vector<uint16_t> &values);

    /**
     * 写输入寄存器，由于这个是直接写内存所以支持写操作，原子操作
     * @param address 起始地址 相对地址
     * @param values 需要写入的值
     */
    void writeRelativeInputRegisters(uint16_t address, std::vector<uint16_t> &values);

    /**
     * 读取保持寄存器数据，读取操作是原子操作
     * @param address 起始地址 绝对地址
     * @param count 读取个数
     * @return 寄存器值的拷贝
     */
    std::vector<uint16_t> readHoldRegisters(uint16_t address, uint16_t count = 1);

    /**
     * 写保持寄存器，原子操作
     * @param address 起始地址 绝对地址
     * @param values 需要写入的值
     */
    void writeHoldRegisters(uint16_t address, std::vector<uint16_t> &values);

    // 批量操作寄存器时，由调用方自己加锁
    inline void setLock() {mutex_.lock();}
    inline void unlock() { mutex_.unlock();}

 private:
    explicit RegisterAdmin();

    inline int startBits() const { return mb_mapping_->start_bits; }
    inline int startInputBits() const { return mb_mapping_->start_input_bits; }
    inline int startInputRegisters() const { return mb_mapping_->start_input_registers; }
    inline int startHoldingRegisters() const { return mb_mapping_->start_registers; }

 private:
    std::mutex mutex_;

    modbus_mapping_t *mb_mapping_ = NULL;
};

}  // namespace core
}  // namespace sros

#endif  // SROS_REGISTER_ADMIN_H
