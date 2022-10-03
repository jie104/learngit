#include <ros/ros.h>
#include "error_detector.hpp"
#include <glog/logging.h>


detector::GoodDetector<const sensor_msgs::LaserScan::ConstPtr> charging_detector;
detector::Goodinfo charge_station_info;
std::vector<detector::GoodDetectInfo> detect_info;

void result(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    detect_info.clear();
    bool is_find = false;
    is_find = charging_detector.findGoodStation(msg, charge_station_info, detect_info);
    LOG(INFO) << "是否发现货物： " << is_find;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "charge");
    ros::NodeHandle n;
    ROS_INFO("hello_world");

    // ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/laser_scan", 100, result);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, result);
    ros::spin();
    return 0;
}