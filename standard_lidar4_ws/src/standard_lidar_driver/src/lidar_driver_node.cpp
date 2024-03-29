//
// Created by lfc on 2022/1/20.
//
#define ROS_NODE

#include <ros/ros.h>
#include "standard_lidar_protocol/wanji_laser_protocol.hpp"
#include "standard_lidar_protocol/wanji_laser_protocol_udp.hpp"

#include "standard_lidar_protocol/siminics_laser_protocol.hpp"
#include "standard_lidar_protocol/keli_laser_protocol.hpp"
#include "standard_lidar_protocol/tcp_scan_data_receiver.hpp"
#include "standard_lidar_protocol/udp_scan_data_receiver.hpp"
#include "standard_lidar_protocol/oradar_laser_protocol_new.hpp"
#include <Eigen/Dense>

using namespace std;

int main(int argc, char **argv) {

//    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
//    Eigen::Vector3f delta_pos = Eigen::Vector3f::Zero();
//    cout << delta_pos;
//    LOG(INFO) << delta_pos;

    ros::init(argc, argv, "ORADAR",ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

//    /// siminics lidar
//  std::shared_ptr<sros::SiminicsLaserProtocol> siminics(new sros::SiminicsLaserProtocol);
////  sros::TcpScanDataReceiver receiver(siminics, "10.10.10.121", 2368);
//// 标定192.168.71.3
//  sros::TcpScanDataReceiver receiver(siminics, "192.168.71.3", 2368);

    /// oradar  lidar
    std::shared_ptr<sros::OradarLaserProtocol> oradar(new sros::OradarLaserProtocol());
//    sros::UdpScanDataReceiver receiver(oradar, "192.168.1.100", 2007); // 雷达IP 192.168.1.100
//    sros::UdpScanDataReceiver receiver(oradar, "192.168.71.90", 2007);
    // aobo换了新雷达，改驱动
//    sros::UdpScanDataReceiver receiver(oradar, "192.168.1.100", 2007);
    // windows上位机改IP，192.168.71.2
    sros::UdpScanDataReceiver receiver(oradar, "192.168.71.2", 2007); // 雷达IP 192.168.71.2 和倍加福雷达IP在同一段.安装的雷达
//    sros::UdpScanDataReceiver receiver(oradar, "192.168.23.100", 2007); // 另一个雷达IP 192.168.23.100,IP改不了 上位机连不上

///////////    sros::UdpScanDataReceiver receiver(oradar, "192.168.71.100", 2007);

    //科力雷达型号“KLM-2027DE”对应扫描范围276°，型号"KLM-2036DE"对应扫描范围360°

    /// wanji lidar
//    std::shared_ptr<sros::WanjiLaserProtocol> wanJi(new sros::WanjiLaserProtocol());
//    sros::TcpScanDataReceiver receiver(wanJi, "192.168.71.101", 2110); // tcp 2.5Hz

//    std::shared_ptr<sros::WanjiLaserProtocolUDP> wanJi(new sros::WanjiLaserProtocolUDP());
//    sros::UdpScanDataReceiver receiver(wanJi, "192.168.71.101", 2110);



    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("orad_scan", 1000);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto last_t1 = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
        std::shared_ptr<ScanMsg> scan;
        auto t2 = std::chrono::high_resolution_clock::now();
        ///  消耗 0.178529 s
        if (receiver.getScan(scan)) {
            // LOG(INFO) << "get scan!";
        }
        auto t22 = std::chrono::high_resolution_clock::now();
        auto elased_time2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t22 - t2).count()/1.0e9;
//        cout << "duration t22: " << elased_time2 << endl; // 0.178529 s

        if(scan) {
            scan->header.frame_id = "scan";
            scan_pub.publish(*scan);
        }


        ros::spinOnce(); // 在while循环中，使用ros::spinOnce()

        t1 = std::chrono::high_resolution_clock::now();
        auto elased_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - last_t1).count()/1.0e9; // 纳秒级别,输出的elased_time单位是秒
        cout << "elased_time = " << elased_time << "s" << endl; // 0.157394 s
        cout << "f = " << 1/elased_time << "Hz" << endl;
        last_t1 = t1;
    }

    return 0;
}
