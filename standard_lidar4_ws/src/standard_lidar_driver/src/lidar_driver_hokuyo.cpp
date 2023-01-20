//
// Created by lfc on 2022/1/20.
//
#define ROS_NODE

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include "standard_lidar_protocol/UST10LX/urg_c_wrapper.h"
#include <memory>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "HOKUYO",ros::init_options::AnonymousName);
    ros::NodeHandle nh;


    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/hokuyo_node/scan", 1000);

    std::shared_ptr<urg_node::URGCWrapper> urg_wrapper(new urg_node::URGCWrapper("192.168.71.5", 10940, true, false));

    urg_wrapper->setAngleLimitsAndCluster(-2.355,2.355,1);
    urg_wrapper->setSkip(1);
    urg_wrapper->start();

    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan);

    while (ros::ok()) {
        // LOG(INFO) << "begin to read!!!";
        urg_wrapper->grabScan(scan);

        if(scan) {
            scan->header.frame_id = "scan";
            scan_pub.publish(*scan);
        }
        usleep(5e4);

        ros::spinOnce(); // 在while循环中，使用ros::spinOnce()

    }

    return 0;
}
