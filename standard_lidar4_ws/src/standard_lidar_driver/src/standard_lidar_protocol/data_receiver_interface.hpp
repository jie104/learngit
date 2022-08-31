//
// Created by lfc on 2022/2/11.
//

#ifndef SROS_DATA_RECEIVER_INTERFACE_HPP
#define SROS_DATA_RECEIVER_INTERFACE_HPP

#ifdef ROS_NODE
#include <sensor_msgs/LaserScan.h>
typedef sensor_msgs::LaserScan ScanMsg;
#else
#include <core/msg/laser_scan_msg.hpp>
typedef sros::core::LaserScanMsg ScanMsg;
#endif
namespace sros {

class DataReceiverInterface {
 public:
    DataReceiverInterface(){

    }

    virtual ~DataReceiverInterface(){

    }

    virtual bool getScan(std::shared_ptr<ScanMsg>& scan) = 0;

    //-----------------------------------------------------------------------------
    virtual void disconnect() = 0;

    virtual bool isConnected() const = 0;

    virtual bool checkConnection() = 0;
};
}
#endif  // SROS_DATA_RECEIVER_INTERFACE_HPP
