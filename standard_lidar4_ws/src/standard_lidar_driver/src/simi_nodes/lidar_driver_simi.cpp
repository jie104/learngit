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

//    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
//    Eigen::Vector3f delta_pos = Eigen::Vector3f::Zero();
//    cout << delta_pos;
//    LOG(INFO) << delta_pos;
    //AnonymousName允许多个相同节点运行
    ros::init(argc, argv, "SIMI",ros::init_options::AnonymousName); 
    ros::NodeHandle nh("~");
    std::string ip;
    int port;
    nh.param<std::string>("ip",ip,"192.168.71.100");
    nh.param<int>("port",port,6060);

/// 要同时运行两个雷达的驱动，新建了这个node节点(.cpp文件)
    /// siminics lidar
    std::shared_ptr<sros::SiminicsLaserProtocol> siminics(new sros::SiminicsLaserProtocol);
//  sros::TcpScanDataReceiver receiver(siminics, "10.10.10.121", 2368);
// 标定192.168.71.3
    sros::TcpScanDataReceiver receiver(siminics, ip, port);

    /// oradar  lidar
//    std::shared_ptr<sros::OradarLaserProtocol> oradar(new sros::OradarLaserProtocol());
////    sros::UdpScanDataReceiver receiver(oradar, "192.168.1.100", 2007); // 雷达IP 192.168.1.100
//    sros::UdpScanDataReceiver receiver(oradar, "192.168.71.90", 2007); // 雷达IP 192.168.71.90 和倍加福雷达IP在同一段

//    sros::UdpScanDataReceiver receiver(oradar, "192.168.71.100", 2007);

    //科力雷达型号“KLM-2027DE”对应扫描范围276°，型号"KLM-2036DE"对应扫描范围360°

    /// wanji lidar
//    std::shared_ptr<sros::WanjiLaserProtocol> wanJi(new sros::WanjiLaserProtocol());
//    sros::TcpScanDataReceiver receiver(wanJi, "192.168.71.101", 2110); // tcp 2.5Hz

//    std::shared_ptr<sros::WanjiLaserProtocolUDP> wanJi(new sros::WanjiLaserProtocolUDP());
//    sros::UdpScanDataReceiver receiver(wanJi, "192.168.71.101", 2110);

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/simi_node/scan", 1000);


    // 让循环按照这个频率执行，然后通过睡眠度过一个循环中剩下的时间，来达到该设定频率，如果能够达到该设定频率则返回true，不能则返回false
//    ros::Rate r(7); // 10Hz,  5Hz-true,


//    auto t1 = std::chrono::high_resolution_clock::now();
//    auto last_t1 = std::chrono::high_resolution_clock::now();


    while (ros::ok()) {

        // 如果程序要使用多个指向同一个对象的指针，应选择shared_ptr. 仅当最后一个指针过期时，才调用delete。
        std::shared_ptr<ScanMsg> scan; // 指向 ScanMsg类型的。
//        auto t2 = std::chrono::high_resolution_clock::now();
        ///  消耗 0.178529 s
        if (receiver.getScan(scan)) {
            // LOG(INFO) << "get scan!";
        }
//        auto t22 = std::chrono::high_resolution_clock::now();
//        auto elased_time2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t22 - t2).count()/1.0e9;
//        cout << "duration t22: " << elased_time2 << endl; // 0.178529 s

        if(scan) {
            scan->header.frame_id = "scan";
            scan_pub.publish(*scan);


//            ros::Time pubTime = ros::Time::now(); // 当前时间，单位s
//            double t_cur = pubTime.toSec(); // 获取的是自1970年一月一日到现在时刻的秒数
////            cout << "pubTime = " << pubTime << endl;
////            cout << "t_cur = " << t_cur << endl;
//
//            ros::Time scanTime = scan->header.stamp; // 这一帧激光的时间戳，s
////            cout << "scanTime = " << scanTime << endl;
//            ros::Duration deltaTime = pubTime - scanTime; // 8000--0.0003s
//            cout << "时间延迟： " << deltaTime << endl; // 0.0003s = 0.3ms
//            ROS_INFO("publish scan...");
        }

//        ros::Time t1= ros::Time::now(); // 单位是s
//        usleep(1e6); // 1e6 us = 1s
//        ros::Time t2 = ros::Time::now();
//        cout << "t2 - t2 = " << t2 - t1 << endl; // 1.000

/*
 * 什么时候用ros::spin()和ros::spinOnce()呢，如果仅仅只是响应topic，就用ros::spin()。
 * 当程序中除了响应回调函数还有其他重复性工作的时候，那就在循环中做那些工作，然后调用ros::spinOnce()。
 * */
        ros::spinOnce(); // 在while循环中，使用ros::spinOnce()


//        usleep(1e4); // 激光雷达帧率，和时间延迟没有关系.   1e4 us = 10ms
//        // usleep(1e6); // Too many scans in receiver queue: Dropping scans!

//        bool met = r.sleep(); // 10Hz
////        cout << "met = " << met << endl; // 如果能够达到该设定频率则返回true，不能则返回false
//        ROS_INFO("met = %d", met);


        // 调用C++11 std::chrono时间库，在代码中可以用来打高精度时间戳
//        t1 = std::chrono::high_resolution_clock::now();
//        auto elased_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - last_t1).count()/1.0e9; // 纳秒级别,输出的elased_time单位是秒
//        cout << "elased_time = " << elased_time << "s" << endl; // 0.157394 s
//        cout << "f = " << 1/elased_time << "Hz" << endl;
////        ROS_INFO_STREAM("f = " << 1/elased_time << "Hz" << endl);
//        last_t1 = t1;

    }

    return 0;
}
