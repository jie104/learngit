//
// Created by lfc on 17-7-17.
//

#ifndef SROS_GAZEBO_LASER_DEVICE_H
#define SROS_GAZEBO_LASER_DEVICE_H
#include "connection.h"
#include "boost/function.hpp"
#include "protocol/gazebo_laser_scan.hpp"
namespace src{

typedef boost::function<void(network::GazeboLaserScanMsg_ptr)> LaserCallback;
class GazeboLaserDevice {
public:
    GazeboLaserDevice():enable_udp(false){

    }

    virtual ~GazeboLaserDevice(){
        disconnect();
    }

    bool connect(std::string src_ip, int src_port, int timeout);

    void disconnect();

    void onNewMsgCallback(network::BaseMsg_ptr m);

    void enableUDP(){
        enable_udp = true;
    }

    void setLaserCallback(LaserCallback callback){
        laserCallback = callback;
    }

private:
    std::shared_ptr<network::Connection> connection_;
    bool enable_udp;

    LaserCallback laserCallback;
};
}



#endif //SROS_GAZEBO_LASER_DEVICE_H
