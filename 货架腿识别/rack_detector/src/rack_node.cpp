//
// Created by lfc on 2020/2/26.
//
#include <glog/logging.h>
#include "ros/ros.h"
#include "rack_detector_operator.hpp"
int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);
    google::InstallFailureSignalHandler();
    ros::init(argc, argv, "rack_detector");  // ros初始化,注释掉部分为tsdfslam,当前启用的是前后帧激光里程计
    rack_detection::RackDetectorOperator detector;

    ros::spin();  //轮寻查看数据，将该程序挂载到ros数据通讯服务器上
    return 0;
}
