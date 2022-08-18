/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#ifndef SROS_RFID_PROTOCOL_INTERFACE_H
#define SROS_RFID_PROTOCOL_INTERFACE_H

#include <cstdint>
#include <vector>
#include <memory>
#include <thread>
#include <future>
#include "glog/logging.h"

namespace ac {

class RfidProtInterface {
public:
    RfidProtInterface() {}
    virtual ~RfidProtInterface() {}

    virtual bool init() = 0;

    virtual void parseMsg(std::vector<uint8_t>& data) = 0;

    virtual std::string getRfid() = 0;

    virtual void asyncGetRfidCmd() {}

    virtual std::string asyncGetRfidData() {return ""; }

    std::string vecToString(std::vector<uint8_t>& raw_data);

protected:
    std::vector<uint8_t> send_raw_data_;    //发数据
    std::vector<uint8_t> recv_raw_data_;    //收数据

    std::shared_ptr<std::promise<std::vector<uint8_t>>> promise_ptr_ = nullptr; //同步
    uint64_t sync_wait_time_ = 2000;  //2秒

};

}


#endif //SROS_RFID_PROTOCOL_INTERFACE_H