/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#ifndef SROS_RFID_MANAGER_H
#define SROS_RFID_MANAGER_H

#include <string>
#include <vector>
#include <memory>
#include "core/usart/connection.hpp"
#include "rfid_protocol_interface.h"

namespace ac {

#define MFRS_GUANGZHOU_WYUAN "WYuan"   //广州网源
#define MFRS_PEPPERL_FUCHS   "PEPPERL_FUCHS"     //倍加福


class RfidManager {
public:
    static RfidManager* getInstance();
     ~RfidManager() {}

    bool init(std::string uart_name, unsigned int baud_rate);
    bool isConnected() { return connected_; }

    bool sendData(const std::vector<uint8_t>& data);

    std::string syncGetRfid();

    void asyncGetRfidCmd();
    std::string asyncGetRfidData();

private:
    RfidManager() {};

    bool connect(std::string uart_name, unsigned int baud_rate, bool rs485_mode = false);

    bool disconnect();

    void onRecvSerialData(const char* data, size_t size);

private:
    std::string rfid_mfrs_name_;

    bool connected_ = false;      //标记是否链接上

    typedef std::shared_ptr<CallbackAsyncSerial> SerialDevice_ptr;
    SerialDevice_ptr serial_ptr_ = nullptr;   //串口设备

    std::shared_ptr<RfidProtInterface> rfid_prot_interface_ptr_ = nullptr; //协议解析
    
};

}


#endif //SROS_RFID_MANAGER_H