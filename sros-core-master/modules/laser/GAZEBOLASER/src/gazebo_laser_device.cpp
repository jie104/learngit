//
// Created by lfc on 17-7-17.
//

#include "gazebo_laser_device.h"
#include "protocol/all_msg.h"

namespace src{
using namespace network;

bool GazeboLaserDevice::connect(std::string src_ip, int src_port, int timeout) {
    connection_.reset(new network::Connection(enable_udp));
    connection_->setMsgCallback(boost::bind(&GazeboLaserDevice::onNewMsgCallback, this, _1));
    return connection_->connect(src_ip.c_str(), src_port);
}

void GazeboLaserDevice::disconnect() {
    if (connection_ && connection_->isNetworkOkay()) {
        connection_->disconnect();
    }
}

void GazeboLaserDevice::onNewMsgCallback(network::BaseMsg_ptr m) {
    switch (m->getType()) {
        case MSG_STATE: {
//            std::cout << "[sdk] got MSG_STATE" << std::endl;
            StateMsg_ptr msg = std::dynamic_pointer_cast<StateMsg>(m);

            break;
        }
        case network::MSG_POSE: {

            break;
        }
        case network::MSG_GAZEBO_LASER_SCAN: {
            if (laserCallback) {
                auto msg = std::dynamic_pointer_cast<network::GazeboLaserScanMsg>(m);
                laserCallback(msg);
            }
            break;
        }
        case network::MSG_SIGNAL: {
            auto msg = std::dynamic_pointer_cast<network::SignalMsg>(m);
            break;
        }
        case network::MSG_INFO: {
            auto msg = std::dynamic_pointer_cast<network::InfoMsg>(m);
            break;
        }
        case network::MSG_USART_DATA: {
            auto msg = std::dynamic_pointer_cast<network::USARTDataMsg>(m);
            break;
        }
        default:
            break;
    }

}


}