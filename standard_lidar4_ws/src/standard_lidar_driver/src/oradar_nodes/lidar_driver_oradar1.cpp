//
// Created by lfc on 2022/1/20.
//
#define ROS_NODE

#include <ros/ros.h>
#include "../standard_lidar_protocol/wanji_laser_protocol.hpp"
#include "../standard_lidar_protocol/wanji_laser_protocol_udp.hpp"

#include "../standard_lidar_protocol/siminics_laser_protocol.hpp"
#include "../standard_lidar_protocol/keli_laser_protocol.hpp"
#include "../standard_lidar_protocol/tcp_scan_data_receiver.hpp"
#include "../standard_lidar_protocol/udp_scan_data_receiver.hpp"
#include "../standard_lidar_protocol/oradar_laser_protocol.hpp"
#include <Eigen/Dense>

using namespace std;

int main(int argc, char **argv) {


    ros::init(argc, argv, "ORADAR_1",ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    std::string ip;
    int port;
    nh.param<std::string>("ip",ip,"192.168.71.102");
    nh.param<int>("port",port,2007);

    std::shared_ptr<sros::OradarLaserProtocol> oradar(new sros::OradarLaserProtocol());
    sros::UdpScanDataReceiver receiver(oradar, ip, port); // 雷达IP 192.168.71.2 和倍加福雷达IP在同一段.安装的雷达

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/oradar_node_1/scan", 1000);

    while (true) {
        std::shared_ptr<ScanMsg> scan;
        if (receiver.getScan(scan)) {
//             LOG(INFO) << "get scan!";
//            LOG(INFO) << scan->ranges.size();
//            for (auto &x:scan->ranges){
//                LOG(INFO) << x << std::endl;

        }
        if(scan) {
            scan->header.frame_id = "scan";
            scan_pub.publish(*scan);
        }

        ros::spinOnce(); // 在while循环中，使用ros::spinOnce()

    }

    return 0;
}
