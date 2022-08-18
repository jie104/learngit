//
// Created by lfc on 17-6-5.
//

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "bag_module.h"
#include "bag_pub.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_pub");
    bag::BagPub bagpub;
//    std::string path = "/sros/map/";
//
//    std::string name = "20160824020ut";

//    bagpub.setBagPath(path);
//    bagpub.setBagName(name);
    bagpub.startPlayBag();

    ros::spin();
    return 0;
}