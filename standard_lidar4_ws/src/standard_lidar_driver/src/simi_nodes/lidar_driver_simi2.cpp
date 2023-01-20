//
// Created by zy on 22-7-25.
//
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
#include <Eigen/Dense>

using namespace std;

int main(int argc, char **argv) {


    ros::init(argc, argv, "SIMI2",ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    std::string ip;
    int port;
    nh.param<std::string>("ip",ip,"192.168.71.103");
    nh.param<int>("port",port,2368);

    std::shared_ptr<sros::SiminicsLaserProtocol> siminics(new sros::SiminicsLaserProtocol);
    sros::TcpScanDataReceiver receiver(siminics, ip, port);

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/simi_node_2/scan", 1000);

    while (ros::ok()) {

        // 如果程序要使用多个指向同一个对象的指针，应选择shared_ptr. 仅当最后一个指针过期时，才调用delete。
        std::shared_ptr<ScanMsg> scan; // 指向 ScanMsg类型的。
        ///  消耗 0.178529 s
        if (receiver.getScan(scan)) {
            // LOG(INFO) << "get scan!";
        }

        if(scan) {
            scan->header.frame_id = "scan";
            scan_pub.publish(*scan);
        }

        ros::spinOnce(); // 在while循环中，使用ros::spinOnce()

    }

    return 0;
}
