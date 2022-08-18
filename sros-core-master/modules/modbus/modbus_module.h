/**
 * @file modbus_module.cpp
 *
 * @author pengjiali
 * @date 18-10-24.
 *
 * @describe modbus通信类 1.https://github.com/stephane/libmodbus/blob/master/tests/bandwidth-server-many-up.c
 * 2. http://velep.com/archives/1137.html
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef MODULES_MODBUS_MODBUS_MODULE_H_
#define MODULES_MODBUS_MODBUS_MODULE_H_

#include <thirty-party/libmodbus/modbus.h>
#include "../communication/communication_module.h"
#include "core/modbus/register_admin.h"
#include "core/msg/base_msg.h"

#define SERVER_ID 17
#define SELECT_TIMEOUT 5            // select的timeout seconds
//const uint16_t MODBUS_VERSION = 1;  // modbus 协议版本号

namespace sros {

enum Modbus_Type {
    MODBUS_TCP,
    MODBUS_TCP_PI,
    MODBUS_RTU,
};

class ModbusException : public std::exception {
 public:
    explicit ModbusException(int exception_id) : exception_id_(exception_id) {}

    int exception_id_;  // 对于 modbus.h 第116行 Protocol exceptions
};

class ModbusModule : public CommunicationModule {
 public:
    explicit ModbusModule(Modbus_Type modbus_type);
    ~ModbusModule() override;

    bool subClassRunPrepare() override;

 private:
    bool init();
    void startASelfCycle();
    void onSelfCycle(core::base_msg_ptr);
    void onHandleRegister(core::base_msg_ptr msg);

    void replyModbusMsg(int req_length);  // 读写一次数据
    bool doConnect();
    void doDisconnect();

    void readDiscreteInputs();
    void readInputRegister();

    void writeSingleCoil();
    void writeMultiCoils();
    void writeSingleRegister();
    void writeMultiRegister();

    void doWriteSingleCoil(uint16_t addr, uint16_t value);
    void doWriteSingleRegister(uint16_t addr, uint16_t value);

    sros::core::TaskNo_t getValue32FormQuery(uint16_t index);
    core::Pose getPoseFormQuery(uint16_t index);

    bool setCurMapUint16(uint16_t value);

    Modbus_Type use_backend_;  // 通信方式

    std::string device_name_rtu_;  // 串口名
    int baud_rate_ = 115200;       // 波特率
    int port_ = 502;               // 端口号
    modbus_t *ctx_ = nullptr;
    core::RegisterAdmin *reg_admin_ = nullptr;
    uint8_t *query_ = nullptr;
    int header_length_;       // modbus协议的头部长度
    bool connected_ = false;  // 标记是否连上，串口专用
    int server_socket_ = -1;  // 服务器监听所用的socket
    fd_set ref_set_;           // 记录的原始set
    fd_set rd_set_;            // 读socket的set
    /* Maximum file descriptor number */
    int fd_max_;
    timeval timeout_;
    std::map<int, uint64_t> map_socket_session_;  // <socket_fd, session_id> 用于适配SessionManager

    core::ObstacleAvoidPolicy obstacle_avoid_policy_ = core::OBSTACLE_AVOID_WAIT;  //  避障策略，TODO：保存到配置？？？
};

}  // namespace sros

#endif  // MODULES_MODBUS_MODBUS_MODULE_H_
