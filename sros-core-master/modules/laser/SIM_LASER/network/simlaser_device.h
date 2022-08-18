//
// Created by lfc on 17-8-31.
//

#ifndef SROS_SIMLASER_DEVICE_H
#define SROS_SIMLASER_DEVICE_H
#include <boost/function.hpp>
#include "connection.h"
#include "connect_msg/base_msg.hpp"
namespace laser{
typedef boost::function<void(connection::BaseMsg_ptr)> LaserCallbackFunc;
class SimLaserDevice {
public:
    SimLaserDevice():enable_udp(false){

    }

    virtual ~SimLaserDevice(){
        disconnect();
    }

    bool connect(std::string src_ip, int src_port, int timeout);

    void disconnect();

    void onNewMsgCallback(connection::BaseMsg_ptr m);

    void enableUDP(){
        enable_udp = true;
    }

    void setLaserCallback(LaserCallbackFunc callback){
        laserCallback = callback;
    }

private:
    std::shared_ptr<network::client::Connection> connection_;
    bool enable_udp;

    LaserCallbackFunc laserCallback;
};

}


#endif //SROS_SIMLASER_DEVICE_H
