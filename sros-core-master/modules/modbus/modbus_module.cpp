/**
 * @file modbus_module.cpp
 *
 * @author pengjiali
 * @date 18-10-24.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "modbus_module.h"
#include <core/device/device_manager.h>
#include <ifaddrs.h>
#include <boost/algorithm/clamp.hpp>
#include <sstream>
#include "core/log/run_logger.h"
#include "core/logger.h"
#include "core/map_manager.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/modbus_register_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/task/task_manager.h"
#include "core/mission/mission_manager.h"
#include "core/util/time.h"
#include "core/fault_center.h"

namespace sros {
ModbusModule::ModbusModule(Modbus_Type modbus_type)
    : CommunicationModule(modbus_type == MODBUS_TCP
                              ? "ModbusModule-TCP"
                              : (modbus_type == MODBUS_RTU ? "ModbusModule-RTU" : "ModbusModule-TCP-PI")),
      use_backend_(modbus_type) {}

ModbusModule::~ModbusModule() {
    if (ctx_ != nullptr) {
        modbus_free(ctx_);
    }
    if (query_ != nullptr) {
        free(query_);
    }
}

bool ModbusModule::subClassRunPrepare() {
    LOG(INFO) << "ModbusModule module start running";

    auto &s = sros::core::Settings::getInstance();
    bool enable_modbus_tcp = (s.getValue<std::string>("modbus.enable_modbus_tcp", "False") == "True");
    bool enable_modbus_rtu = (s.getValue<std::string>("modbus.enable_modbus_rtu", "False") == "True");
    device_name_rtu_ = s.getValue<std::string>("modbus.modbus_rtu_device", "/dev/ttyUSB0");
    baud_rate_ = s.getValue<int>("modbus.modbus_rtu_baud_rate", 115200);
    port_ = s.getValue<int>("modbus.modbus_tcp_port", 502);

    if (use_backend_ == MODBUS_TCP && !enable_modbus_tcp) {
        LOG(WARNING) << "ModbusModule modbus tcp stop running(disable)";
        stop();
        return false;
    }
    if (use_backend_ == MODBUS_RTU) {
        if (!enable_modbus_rtu) {
            LOG(WARNING) << "ModbusModule modbus rtu stop running(disable)";
            stop();
            return false;
        }

        auto enable_touch_screen = (s.getValue<std::string>("hmi.enable_touch_screen", "False") == "True");
        auto touch_screen_device_name = s.getValue<std::string>("hmi.touch_screen_device_name", "/dev/ttyTHS1");
        if (enable_touch_screen && device_name_rtu_ == touch_screen_device_name) {
            LOGGER(ERROR, MODBUS) << "ModbusModule Settings of device_name conflict with touchScreen,"
                                     " modbus_rtu will not start!";
            stop();
            return false;
        }

        auto enable_lmns_usart = (s.getValue<std::string>("main.enable_lmns_usart", "False") == "True");
        auto lmns_device_name = s.getValue<std::string>("lmns.serial_device", "/dev/ttyTHS1");
        if (enable_lmns_usart && device_name_rtu_ == lmns_device_name) {
            LOGGER(ERROR, MODBUS) << "ModbusModule Settings of device_name conflict with lmns usart,"
                                     " modbus_rtu will not start!";
            stop();
            return false;
        }
    }

    if (!init()) {
        LOGGER(ERROR, MODBUS) << "ModbusModule init failed, modbus_rtu will not start!";

        stop();
        return false;
    }

    subscribeTopic("MODBUS_SELF_CYCLE", CALLBACK(&ModbusModule::onSelfCycle));
    subscribeTopic("MODBUS_REGISTER_ACTION", CALLBACK(&ModbusModule::onHandleRegister));

    startASelfCycle();

    LOGGER(INFO, MODBUS) << "ModbusModule module start succeed!";

    return true;
}

bool ModbusModule::init() {
    if (use_backend_ == MODBUS_TCP) {
        ctx_ = modbus_new_tcp("0.0.0.0", port_);
        query_ = reinterpret_cast<uint8_t *>(malloc(MODBUS_TCP_MAX_ADU_LENGTH));
    } else if (use_backend_ == MODBUS_TCP_PI) {
        ctx_ = modbus_new_tcp_pi("::0", "port_");
        query_ = reinterpret_cast<uint8_t *>(malloc(MODBUS_TCP_MAX_ADU_LENGTH));
    } else if (use_backend_ == sros::MODBUS_RTU) {
        auto &s = sros::core::Settings::getInstance();
        auto parity_str = s.getValue<std::string>("modbus.modbus_rtu_parity", "NoneParity");
        char parity = 'N';
        if (parity_str == "OddParity") {
            parity = 'O';
        } else if (parity_str == "EvenParity") {
            parity = 'E';
        }
        ctx_ = modbus_new_rtu(device_name_rtu_.c_str(), baud_rate_, parity, 8, 1);
        modbus_set_slave(ctx_, SERVER_ID);
        query_ = reinterpret_cast<uint8_t *>(malloc(MODBUS_RTU_MAX_ADU_LENGTH));
    } else {
        LOGGER(ERROR, MODBUS) << "UNREACHABLE! unknown backend" << use_backend_;  // UNREACHABLE
    }
    header_length_ = modbus_get_header_length(ctx_);

    modbus_set_debug(ctx_, FALSE);

    reg_admin_ = core::RegisterAdmin::getInstance();
    if (reg_admin_->addrMapHandler() == nullptr) {
        LOGGER(ERROR, MODBUS) << "ModbusModule: Failed to allocate the mapping: " << modbus_strerror(errno);

        return false;
    }

    if (use_backend_ == MODBUS_TCP) {
#define NB_CONNECTION 5  // 监听队列最大个数
        server_socket_ = modbus_tcp_listen(ctx_, NB_CONNECTION);
        if (server_socket_ == -1) {
            LOGGER(ERROR, MODBUS) << "ModbusModule: Unable to listen TCP connection! " << modbus_strerror(errno);
            return false;
        }
        LOGGER(INFO, MODBUS) << "ModbusModule: TCP start listen connection!";

        auto &s = sros::core::Settings::getInstance();
        bool enable_tcp_keep_alive = (s.getValue<std::string>("network.enable_tcp_keep_alive", "False") == "True");
        if (enable_tcp_keep_alive) {
            LOG(INFO) << "tcp keep alive enabled!";

            int keepAlive = 1;  // 开启keepalive属性
            // 如该连接在1800秒内没有任何数据往来,则进行探测
            int keepIdle = s.getValue<int>("network.tcp_keepalive_time", 10);
            int keepInterval = s.getValue<int>("network.tcp_keepalive_intvl", 1);  // 探测时发包的时间间隔为3秒
            // 探测尝试的次数.如果第1次探测包就收到响应了,则后几次的不再发.
            int keepCount = s.getValue<int>("network.tcp_keepalive_probes", 1);
            keepIdle = boost::algorithm::clamp(keepIdle, 10, 7200);
            keepInterval = boost::algorithm::clamp(keepInterval, 1, 75);
            keepCount = boost::algorithm::clamp(keepCount, 1, 30);

            setsockopt(server_socket_, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
            setsockopt(server_socket_, SOL_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
            setsockopt(server_socket_, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
            setsockopt(server_socket_, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));
        }

        /* Clear the reference set of socket */
        FD_ZERO(&ref_set_);
        /* Add the server socket */
        FD_SET(server_socket_, &ref_set_);

        /* Keep track of the max file descriptor */
        fd_max_ = server_socket_;
        //        LOG(INFO) << "fd_max_: " << fd_max_;

        timeout_.tv_sec = SELECT_TIMEOUT;
        timeout_.tv_usec = 0;
    }

    return true;
}

bool ModbusModule::doConnect() {
    if (use_backend_ == sros::MODBUS_RTU) {
        int rc = modbus_connect(ctx_);
        auto fault_center = sros::core::FaultCenter::getInstance();
        if (rc == -1) {
            fault_center->addFault(sros::core::FAULT_CODE_MODBUS_RTU_DEVICE_CAN_NOT_OPEN);
            LOGGER(ERROR, MODBUS) << "ModbusModule Unable to connect " << device_name_rtu_ << ", "
                                  << modbus_strerror(errno);
            sleep(10);  // 防止一直发
            return false;
        }
        fault_center->removeFault(sros::core::FAULT_CODE_MODBUS_RTU_DEVICE_CAN_NOT_OPEN);
        LOGGER(INFO, MODBUS) << "ModbusModule RTU connected succeed! device name:" << device_name_rtu_;
    }

    return true;
}

void ModbusModule::doDisconnect() {
    connected_ = false;
    startASelfCycle();
}

void ModbusModule::onSelfCycle(core::base_msg_ptr) {
    //    static uint64_t begin;
    //    uint64_t end1 = sros::core::util::get_time_in_ms();
    //    LOG(INFO) << "Running end1: " << end1 - begin << "ms";
    //    begin = end1;

    if (use_backend_ == sros::MODBUS_RTU) {
        if (!connected_) {  // 没有链接上
            if (!doConnect()) {
                startASelfCycle();
                return;
            } else {
                connected_ = true;
            }
        }

        int rc = 0;
        rc = modbus_receive(ctx_, query_);
        if (rc == 0) {
            LOGGER(ERROR, MODBUS) << "modbus read none!" << modbus_strerror(errno);
            usleep(10 * 1000);
            startASelfCycle();
            return;
        }

        /* The connection is not closed on errors which require on reply such as
       bad CRC in RTU. */
        if (rc == -1) {
            /* Quit */
            LOGGER(ERROR, MODBUS) << "modbus read error!" << modbus_strerror(errno);

            connected_ = false;
            usleep(10 * 1000);
            startASelfCycle();
            return;
        }

        replyModbusMsg(rc);
    } else if (use_backend_ == MODBUS_TCP) {
        rd_set_ = ref_set_;
        if (select(fd_max_ + 1, &rd_set_, nullptr, nullptr, &timeout_) == -1) {
            LOGGER(ERROR, MODBUS) << "modbus server select failure! " << modbus_strerror(errno);
        }

        /* Run through the existing connections looking for data to be
         * read */
        for (int master_socket = 0; master_socket <= fd_max_; master_socket++) {
            if (!FD_ISSET(master_socket, &rd_set_)) {
                continue;
            }

            if (master_socket == server_socket_) {
                /* A client is asking a new connection */
                socklen_t addr_len;
                struct sockaddr_in client_addr;
                int new_fd;

                /* Handle new connections */
                addr_len = sizeof(client_addr);
                memset(&client_addr, 0, sizeof(client_addr));
                new_fd = accept(server_socket_, (struct sockaddr *)&client_addr, &addr_len);
                if (new_fd == -1) {
                    LOGGER(ERROR, MODBUS) << "modbus server accept error! " << modbus_strerror(errno);
                } else {
                    FD_SET(new_fd, &ref_set_);

                    if (new_fd > fd_max_) {
                        /* Keep track of the maximum */
                        fd_max_ = new_fd;
                    }
                    LOGGER(INFO, MODBUS) << "New connection from " << inet_ntoa(client_addr.sin_addr) << ":"
                                         << client_addr.sin_port << " on socket " << new_fd;

                    auto session_item = g_state.modbus_session_manager.addItem("-", inet_ntoa(client_addr.sin_addr),
                                                                               client_addr.sin_port);
                    map_socket_session_[new_fd] = session_item->session_id;
                }
            } else {
                modbus_set_socket(ctx_, master_socket);
                int rc = modbus_receive(ctx_, query_);
                if (rc > 0) {
                    replyModbusMsg(rc);
                } else if (rc == -1) {
                    /* This example server in ended on connection closing or
                     * any errors. */
                    LOGGER(INFO, MODBUS) << "Connection closed on socket " << master_socket;
                    close(master_socket);

                    /* Remove from reference set */
                    FD_CLR(master_socket, &ref_set_);

                    if (master_socket == fd_max_) {
                        fd_max_--;
                    }

                    g_state.modbus_session_manager.removeItem(map_socket_session_[master_socket]);
                    map_socket_session_.erase(master_socket);
                }
            }
        }
    }

    startASelfCycle();
    usleep(10 * 1000);
}

void ModbusModule::onHandleRegister(core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<core::ModbusRegisterMsg>(m);
    if (!msg) {
        return;
    }

    uint16_t addr = msg->addr_;
    uint32_t dat = msg->dat_;
    core::ModbusAddrType register_type = (core::ModbusAddrType)(msg->registerType_);

    LOGGER(INFO, MODBUS) << "Handle register" << msg->getOperationStr() << " " << addr << " " << dat;

    try {
        switch (msg->operation_type_) {
            case core::RegisterAction::ActionWrite: {
                switch (register_type) {
                    case core::ModbusAddrType::AddrCoil: {
                        dat = dat ? 0xFF00 : 0x0;  // 非0的都认为是0xFF00
                        doWriteSingleCoil(reg_admin_->mapAddrBits(addr), dat);
                        reg_admin_->writeCoilRegister(addr, dat);
                        break;
                    }
                    case core::ModbusAddrType::AddrHoldingRegister: {
                        doWriteSingleRegister(reg_admin_->mapAddrHoldingRegisters(addr), dat);
                        reg_admin_->writeHoldingRegister(addr, dat);
                        break;
                    }
                    default: {
                        break;
                    }
                }
                break;
            }
            case core::RegisterAction::ActionRead: {
                break;
            }
            default: {
                break;
            }
        }
    } catch (ModbusException e) {
        LOGGER(ERROR, MODBUS) << "exception id is" << e.exception_id_;
    }
}

/**
 * @brief
 * @param req_length
 * @note 现在是读之前写入， 若是后期感觉处理不过来，可以定时更新
 */
void ModbusModule::replyModbusMsg(int req_length) {
    int rc;

    try {
        switch (query_[header_length_]) {
            case MODBUS_FC_READ_DISCRETE_INPUTS: {
                readDiscreteInputs();
                break;
            }
            case MODBUS_FC_WRITE_SINGLE_COIL: {
                writeSingleCoil();
                break;
            }
            case MODBUS_FC_WRITE_MULTIPLE_COILS: {
                writeMultiCoils();
                break;
            }
            case MODBUS_FC_READ_INPUT_REGISTERS: {  // Read Input Register
                readInputRegister();
                break;
            }
            case MODBUS_FC_WRITE_SINGLE_REGISTER: {
                writeSingleRegister();
                break;
            }
            case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
                writeMultiRegister();
                break;
            }
            case MODBUS_FC_READ_COILS:
            case MODBUS_FC_READ_HOLDING_REGISTERS: {
                // NOTE： 读只写的区域时，不报错，但是不处理。
                // 比如触摸屏写入了寄存器后，想继续读刚才写入的东西，这种奇葩的做法。
                break;
            }
            default: {
                LOG(ERROR) << "Function code is " << (int)query_[header_length_];
                throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
                break;
            }
        }
    } catch (const ModbusException &e) {
        rc = modbus_reply_exception(ctx_, query_, e.exception_id_);
        //        LOG(ERROR) << rc;
        LOGGER(ERROR, MODBUS) << "exception id is " << e.exception_id_;
        startASelfCycle();
        return;
    }

    // 用默认的方式回数据
    // 读写寄存器前需要加锁
    reg_admin_->setLock();
    rc = modbus_reply(ctx_, query_, req_length, reg_admin_->addrMapHandler());
    reg_admin_->unlock();
    if (rc == -1) {
        LOGGER(ERROR, MODBUS) << "modbus write error！";
        doDisconnect();
        return;
    }
}

void ModbusModule::readDiscreteInputs() {
    int nb_bits = reg_admin_->nbInputBits();
    uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 1);
    uint16_t quantity = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 3);

    /* The mapping can be shifted to reduce memory consumption and it
       doesn't always start at address zero. */
    int mapping_address = reg_admin_->mapAddrInputBits(startAddr);
    //    LOG(INFO) << "ModbusModule::readDiscreteInputs(): start_addr=" << mapping_address << " quantity=" << quantity;

    if (quantity < 1 || MODBUS_MAX_READ_BITS < quantity) {
    } else if (mapping_address < 0 || (mapping_address + quantity) > nb_bits) {
    } else {
    }
}

void ModbusModule::readInputRegister() {
    uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 1);
    uint16_t quantity = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 3);
    int mapping_address = reg_admin_->mapAddrInputRegisters(startAddr);

    //    LOG(INFO) << "ModbusModule::readInputRegister(): start_addr=" << mapping_address << " quantity=" << quantity;

    // 将需要查询的信息写入内存
    if (quantity < 1 || MODBUS_MAX_READ_REGISTERS < quantity) {
    } else if (mapping_address < 0 || (mapping_address + quantity) > reg_admin_->nbInputRegisters()) {
    } else {
    }
}

void ModbusModule::startASelfCycle() {
    auto mm = std::make_shared<sros::core::CommonMsg>("MODBUS_SELF_CYCLE");
    mm->is_real_time_ = true;
    sendMsg(mm);
}

void ModbusModule::writeSingleRegister() {
    uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 1);
    uint16_t value = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 3);
    int mapping_address = reg_admin_->mapAddrHoldingRegisters(startAddr);

    LOGGER(INFO, MODBUS) << "write single register, start_addr:" << startAddr << " value:" << value;

    // 写入内存前先执行对应的操作
    if (mapping_address < 0 || mapping_address > reg_admin_->nbHoldingRegisters()) {
        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    } else {
        doWriteSingleRegister(mapping_address, value);
    }
}

void ModbusModule::writeSingleCoil() {
    uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 1);
    uint16_t value = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 3);
    int mapping_address = reg_admin_->mapAddrBits(startAddr);

    LOGGER(INFO, MODBUS) << "write single coil, start_addr:" << mapping_address << " value:" << value;

    if (mapping_address < 0 || mapping_address >= reg_admin_->nbBits()) {
        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    } else {
        doWriteSingleCoil(mapping_address, value);
    }
}

void ModbusModule::writeMultiCoils() {
    uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 1);
    uint16_t quantity = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 3);  // 寄存器个数
    int mapping_address = reg_admin_->mapAddrBits(startAddr);

    LOGGER(INFO, MODBUS) << "write Multi coils, start_addr:" << startAddr << " quantity:" << quantity;

    if (quantity < 1 || MODBUS_MAX_WRITE_REGISTERS < quantity) {
        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    } else if (mapping_address < 0 || (mapping_address + quantity) > reg_admin_->nbBits()) {
        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    } else {
        if (quantity == 1) {  // 为兼容有些老plc没有写单个线圈，然后就用写多个线圈功能码写一个来实现兼容
            uint16_t value = query_[header_length_ + 6] == 1 ? 0xFF00 : 0x0000;

            doWriteSingleCoil(mapping_address, value);
        } else {
            throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        }
    }
}

void ModbusModule::writeMultiRegister() {
    uint16_t startAddr = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 1);
    uint16_t quantity = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 3);  // 寄存器个数
    int mapping_address = reg_admin_->mapAddrHoldingRegisters(startAddr);

    LOGGER(INFO, MODBUS) << "write multi register, start_addr:" << startAddr << " quantity:" << quantity;

    if (quantity < 1 || MODBUS_MAX_WRITE_REGISTERS < quantity) {
        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    } else if (mapping_address < 0 || (mapping_address + quantity) > reg_admin_->nbHoldingRegisters()) {
        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
    } else {
        if (quantity == 1) {
            uint16_t value = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 6);

            doWriteSingleRegister(mapping_address, value);
        } else {
            bool ret;
            switch (mapping_address) {
                case HRA_START_LOCATION_POSE: {
                    if (quantity != 6) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    core::Pose pose = getPoseFormQuery(header_length_ + 6);
                    ret = startLocationByPose(pose);
                    break;
                }
                case HRA_MOVE_TO_POSE: {
                    if (quantity != 6) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    core::Pose pose = getPoseFormQuery(header_length_ + 6);
                    ret = moveToPose(0, pose, obstacle_avoid_policy_);
                    break;
                }
                case HRA_SET_ACTION: {
                    if (quantity != 3) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    ret = startNewAction(0, MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 6),
                                         MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 8),
                                         MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 10));
                    break;
                }
                case HRA_MANUAL_CONTROL_V_X: {
                    if (quantity != 3) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    ret = setSRCSpeed(MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 6),
                                      MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 8),
                                      MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 10));
                    break;
                }
                case HRA_SET_GPIO_OUT_VALUE: {
                    if (quantity != 2) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    ret = setOutGPIO(MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 6),
                                     MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 8));
                    break;
                }
                case HRA_START_LOCATION_POSE_FORCE: {
                    if (quantity != 6) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    core::Pose pose = getPoseFormQuery(header_length_ + 6);
                    ret = startLocationByPose(pose, true);
                    break;
                }
                case HRA_PROXY_IO_0:
                case HRA_PROXY_IO_1:
                case HRA_PROXY_IO_2:
                case HRA_PROXY_IO_3:
                case HRA_PROXY_IO_4:
                case HRA_PROXY_IO_5:
                case HRA_PROXY_IO_6:
                case HRA_PROXY_IO_7: {
                    // 写用户寄存器，可以一次写多个
                    if ((mapping_address + quantity) <= (HRA_PROXY_IO_7 + 1)) {
                        ret = true;
                        break;
                    }

                    throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                }

                case HRA_MOVE_TO_POSE_WITH_NO: {
                    if (quantity != 8) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    auto task_no = getValue32FormQuery(header_length_ + 6);
                    core::Pose pose = getPoseFormQuery(header_length_ + 10);
                    ret = moveToPose(task_no, pose, obstacle_avoid_policy_);
                    break;
                }
                case HRA_MOVE_TO_STATION_WITH_NO: {
                    if (quantity != 3) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    auto task_no = getValue32FormQuery(header_length_ + 6);
                    uint16_t station_id = MODBUS_GET_INT16_FROM_INT8(query_, header_length_ + 10);
                    ret = moveToStation(task_no, station_id, obstacle_avoid_policy_);
                    break;
                }
                case HRA_SET_ACTION_WITH_NO: {
                    if (quantity != 8) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    auto task_no = getValue32FormQuery(header_length_ + 6);
                    ret = startNewAction(task_no, getValue32FormQuery(header_length_ + 10),
                                         getValue32FormQuery(header_length_ + 14),
                                         getValue32FormQuery(header_length_ + 18));
                    break;
                }
                case HRA_START_MISSION: {
                    if (quantity != 2) {
                        throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                    }
                    auto mission_id = getValue32FormQuery(header_length_ + 6);
                    ret = startMission(mission_id);
                    break;
                }
                default: {
                    throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                    break;
                }
            }

            // 处理失败的情况
            if (!ret) {
                LOGGER(ERROR, MODBUS) << "write multi register failed!";

                throw ModbusException(MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE);
            }
        }
    }
}

void ModbusModule::doWriteSingleCoil(uint16_t addr, uint16_t value) {
    LOGGER(INFO, MODBUS) << "write single coil, addr:" << addr << " value:" << value;

    bool ret = false;
    switch (addr) {
        case DOC_PAUSE: {
            if (value == 0xFF00) {
                ret = srcPause();
            }
            break;
        }
        case DOC_CONTINUE: {
            if (value == 0xFF00) {
                ret = srcContinue();
            }
            break;
        }
        case DOC_STOP: {
            if (value == 0xFF00) {
                ret = srcCancel();
            }
            break;
        }

        case DOC_STOP_LOCATION: {
            if (value == 0xFF00) {
                ret = stopLocation();
            }
            break;
        }

        case DOC_TRIGGER_EMERGENCY: {
            if (value == 0xFF00) {
                ret = triggerEmergency();
            }
            break;
        }
        case DOC_CANCEL_EMERGENCY: {
            if (value == 0xFF00) {
                ret = cancelEmergency();
            }
            break;
        }
        case DOC_ENABLE_CHARGE: {
            if (value == 0xFF00) {
                ret = startCharge();
            }
            break;
        }
        case DOC_DISENABLE_CHARGE: {
            if (value == 0xFF00) {
                ret = stopCharge();
            }
            break;
        }
        case DOC_ENABLE_POWER_MODE: {
            if (value == 0xFF00) {
                ret = enablePowerMode();
            }
            break;
        }
        case DOC_DISENABLE_POWER_MODE: {
            if (value == 0xFF00) {
                ret = disablePowerMode();
            }
            break;
        }
        case DOC_RESTART_SYSTEM: {
            if (value == 0xFF00) {
                ret = restartSystem();
            }
            break;
        }
        case DOC_ENABLE_MANUAL_CONTROL: {
            if (value == 0xFF00) {
                ret = startManualControl();
            }
            break;
        }
        case DOC_DISENABLE_MANUAL_CONTROL: {
            if (value == 0xFF00) {
                ret = stopManualControl();
            }
            break;
        }
        case DOC_GO_FORWARD: {
            if (value == 0xFF00) {
                ret = goForward();
            }
            break;
        }
        case DOC_GO_BACK: {
            if (value == 0xFF00) {
                ret = goBack();
            }
            break;
        }
        case DOC_GO_LEFT: {
            if (value == 0xFF00) {
                ret = goLeft();
            }
            break;
        }
        case DOC_GO_RIGHT: {
            if (value == 0xFF00) {
                ret = goRight();
            }
            break;
        }
        case DOC_LET_GO: {
            if (value == 0xFF00) {
                ret = inputActionValue();
            }
            break;
        }
        case DOC_FLEET_MODE: {
            if (value == 0xFF00) {
                ret = triggerFleetOnline();
            } else if (value == 0x00) {
                ret = triggerFleetOffline();
            }
            break;
        }
        case DOC_DPIO_OUTPUT_0:
        case DOC_DPIO_OUTPUT_1:
        case DOC_DPIO_OUTPUT_2:
        case DOC_DPIO_OUTPUT_3:
        case DOC_DPIO_OUTPUT_4:
        case DOC_DPIO_OUTPUT_5:
        case DOC_DPIO_OUTPUT_6:
        case DOC_DPIO_OUTPUT_7: {
            if (value == 0xFF00) {
                ret = setOutGPIOBit(addr - DOC_DPIO_OUTPUT_0, true);
            } else if (value == 0x0000) {
                ret = setOutGPIOBit(addr - DOC_DPIO_OUTPUT_0, false);
            }
            break;
        }
        case DOC_PAUSE_MISSION: {
            if (value == 0xFF00) {
                ret = pauseMission();
            }
            break;
        }
        case DOC_CONTINUE_MISSION: {
//            LOG(INFO) << "DOC_CONTINUE_MISSION";
            if (value == 0xFF00) {
                auto mission_manager = sros::core::MissionManager::getInstance();
//                uint64_t uuid = mission_manager->getCurrentRunningMissionUuid();
//                LOG(INFO) << "uuid: " << uuid;
                if(mission_manager->isMissionRunning()) {
                    ret = srcContinue();
                } else {
                    ret = continueMission();
                }
            }
            break;
        }
        case DOC_CANCEL_MISSION: {
            if (value == 0xFF00) {
                auto mission_manager = sros::core::MissionManager::getInstance();
                uint64_t uuid = mission_manager->getCurrentRunningMissionUuid();
                if (uuid <= 0) {
                    ret = false;
                } else {
                    ret = cancelMission(uuid);
                }
            }
            break;
        }
        default: {
            LOGGER(ERROR, MODBUS) << "Invalid register address!";
            throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        }
    }

    // 处理失败的情况
    if (!ret) {
        LOGGER(ERROR, MODBUS) << "write single coils failed!";
        throw ModbusException(MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE);
    }
}

void ModbusModule::doWriteSingleRegister(uint16_t addr, uint16_t value) {
    bool ret = true;
    switch (addr) {
        case HRA_START_LOCATION_STATION: {
            // 不需要等待定位结束后再返回response
            startLocationByStation(value);
            ret = true;
            break;
        }
        case HRA_MOVE_TO_STATION: {
            ret = moveToStation(0, value, obstacle_avoid_policy_);
            break;
        }
        case HRA_SET_AVOID_POLICY: {
            if (value != core::OBSTACLE_AVOID_WAIT && value != core::OBSTACLE_AVOID_REPLAN &&
                value != core::OBSTACLE_AVOID_NONE) {
                throw ModbusException(MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
            }
            LOGGER(INFO, MODBUS) << "Set avoid policy" << value;
            obstacle_avoid_policy_ = (core::ObstacleAvoidPolicy)value;
            break;
        }
        case HRA_SET_SPEED_LEVEL: {
            ret = setSpeedLevel(value);
            break;
        }
        case HRA_SET_CUR_STATION: {
            ret = setCurStation(value);
            break;
        }
        case HRA_VOLUME: {
            ret = setVolume(value);
            break;
        }
        case HRA_SET_CUR_MAP: {
            ret = setCurMapUint16(value);
            break;
        }
        default: {
            break;
        }
    }

    // 处理失败的情况
    if (!ret) {
        LOGGER(ERROR, MODBUS) << "write single register failed!";

        throw ModbusException(MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE);
    }
}

core::Pose ModbusModule::getPoseFormQuery(uint16_t index) {  // 电脑是小端，网络是大端
    char tmp[4];
    uint8_t *data = query_ + index;

    tmp[0] = data[3];
    tmp[1] = data[2];
    tmp[2] = data[1];
    tmp[3] = data[0];
    int pose_x_i = *(reinterpret_cast<int *>(tmp));
    double pose_x = pose_x_i / 1000.0;  // 单位m

    tmp[0] = data[4 + 3];
    tmp[1] = data[4 + 2];
    tmp[2] = data[4 + 1];
    tmp[3] = data[4 + 0];
    int pose_y_i = *(reinterpret_cast<int *>(tmp));
    double pose_y = pose_y_i / 1000.0;  // 单位m

    tmp[0] = data[8 + 3];
    tmp[1] = data[8 + 2];
    tmp[2] = data[8 + 1];
    tmp[3] = data[8 + 0];
    int pose_yaw_i = *(reinterpret_cast<int *>(tmp));
    double pose_yaw = pose_yaw_i / 1000.0;  // 单位rad

    //    LOG(ERROR) << "############################## (x: " << pose_x << ", y: " << pose_y << ", yaw: " << pose_yaw <<
    //    ")";
    return sros::core::Pose(sros::core::Location(pose_x, pose_y), sros::core::Rotation(pose_yaw));
}

sros::core::TaskNo_t ModbusModule::getValue32FormQuery(uint16_t index) {
    char tmp[4];
    uint8_t *data = query_ + index;

    tmp[0] = data[3];
    tmp[1] = data[2];
    tmp[2] = data[1];
    tmp[3] = data[0];
    int value = *(reinterpret_cast<int *>(tmp));
    return value;
}

bool ModbusModule::setCurMapUint16(uint16_t value) {
    auto map_list = sros::core::MapManager::getInstance()->getMapListSortByName();

    std::string whole_map_name;
    for (auto map_name : map_list) {
        if (map_name.size() >= 2) {
            LOGGER(INFO, MODBUS) << "map " << map_name << " start with 0x" << std::hex << (uint16_t)map_name[0]
                                 << (uint16_t)map_name[1];
            if ((uint16_t)map_name[0] == (value >> 8) && (uint16_t)map_name[1] == (value & 0xFF)) {
                whole_map_name = map_name;
                break;
            }
        } else if (map_name.size() == 1) {
            LOGGER(INFO, MODBUS) << "map " << map_name << " start with 0x" << std::hex << (uint16_t)map_name[0] << "00";
            if ((uint16_t)map_name[0] == (value >> 8) && 0x00 == (value & 0xFF)) {
                whole_map_name = map_name;
                break;
            }
        }
    }

    if (whole_map_name.empty()) {
        LOGGER(ERROR, MODBUS) << "Can not find map name start with 0x" << std::hex << value;
        return false;
    }

    LOGGER(INFO, MODBUS) << "Set current map " << whole_map_name;

    return setCurMap(whole_map_name);
}

}  // namespace sros
