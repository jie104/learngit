//
// Created by lfc on 2022/1/20.
//
#define ROS_NODE

#include <ros/ros.h>
#include "standard_lidar_protocol/tcp_scan_data_receiver.hpp"
#include "standard_lidar_protocol/udp_scan_data_receiver.hpp"
#include <eigen3/Eigen/Dense>
#include "standard_lidar_protocol/leimou_laser_protocol.hpp"
// #include "standard_lidar_protocol/keli_laser_protocol.hpp"




using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "LidarDriverNode");
    ros::NodeHandle nh;

//    siminics lidar
//  std::shared_ptr<sros::SiminicsLaserProtocol> siminics(new sros::SiminicsLaserProtocol);
//  sros::TcpScanDataReceiver receiver(siminics, "192.168.71.100", 2368);

    /// oradar  lidar
//    std::shared_ptr<sros::OradarLaserProtocol> oradar(new sros::OradarLaserProtocol());
//    sros::UdpScanDataReceiver receiver(oradar, "192.168.1.100", 2007);

    //科力雷达型号“KLMsta-2027DE”对应扫描范围276°，型号"KLM-2036DE"对应扫描范围360°

//    /// wanji lidar
    std::shared_ptr<sros::LeimouLaserProtocol> leimoui(new sros::LeimouLaserProtocol());
    sros::TcpScanDataReceiver receiver(leimoui, "192.168.71.201", 2007);

//    std::shared_ptr<sros::WanjiLaserProtocolUDP> wanJi(new sros::WanjiLaserProtocolUDP());
//    // sros::UdpScanDataReceiver receiver(wanJi, "192.168.71.101", 2110);
//    ///!! WLR-716内部UDP通信端口固定为6050
//    // 雷达端： IP：客户自定义雷达IP 192.168.71.100,  Port: 6050
//    // WLR-716 同一时刻只能使用一种协议进行通信并且优先使用TCP协议
//    sros::UdpScanDataReceiver receiver(wanJi, "192.168.71.101", 6050);


//  std::shared_ptr<sros::SiminicsLaserProtocol> siminics(new sros::SiminicsLaserProtocol);
//  sros::TcpScanDataReceiver receiver(siminics, "192.168.71.100", 2368);

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 1000);

    while (ros::ok()) {
        std::shared_ptr<ScanMsg> scan;

        if (receiver.getScan(scan)) {
            LOG(INFO) << "get scan!";
        }
        if(scan) {
            scan->header.frame_id = "laser_scan";
            scan_pub.publish(*scan);
        }

        ros::spinOnce();
        usleep(1e4);
    }

    return 0;
}
