//
// Created by lfc on 16-8-20.
//


#include "bag_pub.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <srosbag/msg/pub_msg/laser_scan_msg.hpp>
#include <srosbag/msg/pub_msg/PoseStampedMsg.h>
// #include <glog/logging.h>

void bag::BagPub::setBagName(std::string &bagname) {
    bag_cmd->setBagName(bagname);
}

void bag::BagPub::setBagPath(std::string &bagpath) {
    bag_cmd->setBagPath(bagpath);
}

void bag::BagPub::startPlayBag() {
    srosoutstream << "will start the play!\n";

    bagmodule.handleStartPlay(bag_cmd);
}

bag::BagPub::~BagPub() {


}

bag::BagPub::BagPub() {
    bag_cmd.reset(new bag::BagCommandMsg);
    bag::globalbagconfig.setbagscanCallback(boost::bind(&BagPub::bagscanCallback, this, _1));
    bag::globalbagconfig.setposestampedCallback(boost::bind(&BagPub::bagposeCallback, this, _1));

    keyboard_manager = KeyboardManager::getInstance();

    keyboard_manager->setCmdCallback(boost::bind(&bag::BagPub::handleKeyCmd, this, _1));

    scan_pub = node_.advertise<sensor_msgs::LaserScan>("scan", 5);
    pose_pub = node_.advertise<nav_msgs::Odometry>("odom", 5);
    path_pub = node_.advertise<nav_msgs::Path>("bagpath", 5);

    node_.param<std::string>("/srosbag_pub/bag_path", bag_path, "/sros/feature/");
    setBagPath(bag_path);
    node_.param<std::string>("/srosbag_pub/bag_name", bag_name, "lmk_test");
    setBagName(bag_name);
    node_.param<float>("/srosbag_pub/time_scale", time_scale, 4.0);
    double begin_time;
    node_.param<double>("/srosbag_pub/begin_play_time", begin_time, 0);
    begin_play_time = begin_time * 1.0e6;
    //LOG(INFO) << "play time:" << begin_play_time;
    bag::globalbagconfig.time_scale = 10000000000.0;

    path_vec.poses.clear();
    count = 1;
}

void bag::BagPub::bagscanCallback(sros::bag::base_msg_ptr base_msg) {
//    printf("dddd\n");
//    count++;
//    if (count >= 3) {
//        count = 1;
//    }else {
//        return;
//    }
    if(!laterThanBeginTime(base_msg->time_)){
      return;
    }

    sros::bag::LaserScan_ptr scan_ptr = std::dynamic_pointer_cast<sros::bag::LaserScanMsg>(base_msg);
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "scan";
    double scan_time = (double) scan_ptr->time_ / (1.0e6);
    //    //LOG(INFO)<<"\r\033[k";
//    //LOG(INFO) << "out!";
    std::cerr << "\r";
    std::cerr << "curr play] time:" <<std::setiosflags(std::ios::fixed)<<std::setprecision(8)<< scan_time << std::flush;

    //    printf("\r\033[k");
  scan.header.stamp.fromSec(scan_time);//这里需要再进行一次坐标变换，把

    scan.header.stamp = ros::Time::now();//为了能够仿真
    scan.angle_increment = scan_ptr->angle_increment;
    scan.angle_max = scan_ptr->angle_max;
    scan.angle_min = scan_ptr->angle_min;
    scan.intensities = scan_ptr->intensities;
    scan.range_max = scan_ptr->range_max;
    scan.range_min = scan_ptr->range_min;
    scan.ranges = scan_ptr->ranges;
    scan_pub.publish(scan);


}

void bag::BagPub::bagposeCallback(sros::bag::base_msg_ptr base_msg) {
//    count++;
//    if (count > (int) 3) {
//        count = 1;
//    }else {
//        return;
//    }
    if(!laterThanBeginTime(base_msg->time_)){
      return;
    }
    sros::bag::PoseStamped_ptr odopose_ptr = std::dynamic_pointer_cast<sros::bag::PoseStampedMsg>(base_msg);
    geometry_msgs::PoseStamped bagpose;
    bagpose.header.frame_id = "map";
    double pose_time= (double)odopose_ptr->time_ / (1.0e6);
    bagpose.header.stamp.fromSec(pose_time);
    Eigen::Vector3f scan_pose(0.1, 0.006, 0.0);
    Eigen::Vector3f scan_odo_pose = Eigen::Vector3f::Zero();

    scan_odo_pose[0] = cos(odopose_ptr->pose.yaw()) * scan_pose[0] - sin(odopose_ptr->pose.yaw()) * scan_pose[1] +
                   odopose_ptr->pose.x();
    scan_odo_pose[1] = sin(odopose_ptr->pose.yaw()) * scan_pose[0] + cos(odopose_ptr->pose.yaw()) * scan_pose[1] +
                   odopose_ptr->pose.y();

    scan_odo_pose[2] = scan_pose[2] + odopose_ptr->pose.yaw();
    bagpose.header.stamp = ros::Time::now();//为了能够仿真google carto
    bagpose.pose.position.x = odopose_ptr->pose.x();
    bagpose.pose.position.y = odopose_ptr->pose.y();
    bagpose.pose.position.z = odopose_ptr->pose.z();
    bagpose.pose.orientation.w = cos(odopose_ptr->pose.yaw() * 0.5);
    bagpose.pose.orientation.z = sin(odopose_ptr->pose.yaw() * 0.5);
    nav_msgs::Odometry odom_pu;
    odom_pu.child_frame_id = "base_link";
    odom_pu.header.frame_id = "odom";
    odom_pu.header.stamp = ros::Time::now();
    odom_pu.pose.pose = bagpose.pose;
    pose_pub.publish(odom_pu);

    transform_scan_to_base.child_frame_id_ = "scan";
    transform_scan_to_base.frame_id_ = "base_link";
    transform_scan_to_base.getOrigin().setX(0.285);
    transform_scan_to_base.getOrigin().setY(0.00);
    transform_scan_to_base.getOrigin().setZ(0.0);
    tf::Quaternion quaternion_scan;
    quaternion_scan.setEuler(0.0, 0.0, 0.0);
    transform_scan_to_base.setRotation(quaternion_scan);
    transform_scan_to_base.stamp_ = ros::Time::now();
    tfB_.sendTransform(transform_scan_to_base);

    path_vec.header.stamp.fromSec(pose_time);
    path_vec.header.frame_id = "map";
    path_vec.poses.push_back(bagpose);
    path_pub.publish(path_vec);
}

void bag::BagPub::handlePause() {
    //LOG(INFO) << "will pause!";
    bagmodule.handlePausePlay(bag_cmd);

}

void bag::BagPub::handleContinue() {
    //LOG(INFO) << "will continue!";
    bagmodule.handleContinuePlay(bag_cmd);
}

void bag::BagPub::handleKeyCmd(keyboard::KeyCmdType type) {
    using namespace keyboard;

    switch (type) {
        case TYPE_CMD_PAUSE:
            handlePause();
            break;
        case TYPE_CMD_CONTINUE:
            handleContinue();
            break;
        default:
            //LOG(INFO) << "will not handle the cmd!" << type;
            break;
    }
}
bool bag::BagPub::laterThanBeginTime(int64_t curr_time) {
  if (curr_time > begin_play_time) {
    bag::globalbagconfig.time_scale = time_scale;
    return true;
  }
  return false;
}
