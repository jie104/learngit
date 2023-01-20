//
// Created by lfc on 18-10-18.
//

#include <ros/ros.h>
#include "tail_filter.hpp"
#include "target_detector/charging_station_detector.hpp"
#include <glog/logging.h>
//#include "laser_filter.hpp"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);
    google::InstallFailureSignalHandler();

    LOG(INFO) << "hello world!";

    ros::init(argc, argv, "laser_filter_node");//ros初始化

    TailFilter laser_filter;

    ros::spin();
}