//
// Created by lfc on 17-8-31.
//

#include "simlaser_device.h"
#include <boost/function.hpp>
#include "connection.h"
namespace laser{

bool SimLaserDevice::connect(std::string src_ip, int src_port, int timeout) {
    connection_.reset(new network::client::Connection(enable_udp));
    connection_->setMsgCallback(boost::bind(&SimLaserDevice::onNewMsgCallback, this, _1));
    connection_->setDisconnectCallback(boost::bind(&SimLaserDevice::disconnect, this));
    return connection_->connect(src_ip.c_str(), src_port);
}

void SimLaserDevice::disconnect() {
    if (connection_ && connection_->isNetworkOkay()) {
        connection_->disconnect();
    }
}

void SimLaserDevice::onNewMsgCallback(connection::BaseMsg_ptr m) {
    if (laserCallback) {
        laserCallback(m);
    }
}
}