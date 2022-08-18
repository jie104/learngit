//
// Created by john on 19-5-20.
//

#ifndef SROS_CANBUS_H
#define SROS_CANBUS_H

#include <mutex>
#include <map>
#include "socket_can.hpp"
#include "core/device/IOInterface.hpp"

class Canbus {
    Canbus();

public:
    static Canbus &getInstance();

    void addDevice(canid_t response_id, sros::device::IOInterfaceReadDataFunc func);

    void sendData(canid_t can_id, const std::vector<uint8_t>& data);
private:
    void runCan();

    void onCanMsgCallback(canid_t can_id, const std::vector<uint8_t> &data);

    enum class STATE {
        INITIALIZING,
        RUNNING,
        STOPPED,
    };
    STATE state_ = STATE::INITIALIZING;

    std::mutex mutex_can_;
    std::map<canid_t, sros::device::IOInterfaceReadDataFunc> read_callback_map_; // <response can id, callback>
    SocketCan_ptr can_;
    std::string device_name_;
};


#endif //SROS_CANBUS_H
