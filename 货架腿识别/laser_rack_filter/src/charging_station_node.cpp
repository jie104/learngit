//
// Created by lfc on 19-2-15.
//

#include <ros/ros.h>
#include <glog/logging.h>
//#include "laser_filter.hpp"
//#include "tail_filter.hpp"
#include "target_detector/charging_station_wrap.hpp"
int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);
    google::InstallFailureSignalHandler();
    ros::init(argc, argv, "charging_station_node");//ros初始化

    ChargingStationWrap station_wrap;

    ros::spin();
}