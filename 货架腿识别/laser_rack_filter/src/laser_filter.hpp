//
// Created by lfc on 18-10-18.
//

#ifndef PROJECT_LASER_FILTER_HPP
#define PROJECT_LASER_FILTER_HPP

#include <ros/ros.h>
#include <glog/logging.h>
#include "laser_filter/rack_legs_filter.hpp"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
class LaserFilter {
public:
    LaserFilter(){
        scan_sub = node.subscribe("scan", 5, &LaserFilter::scanCallback, this);
        rectangle_pub = node.advertise<sensor_msgs::LaserScan>("region_point", 3);
        rectangle_pub_1 = node.advertise<sensor_msgs::LaserScan>("region_point_1", 3);
        rectangle_pub_2 = node.advertise<sensor_msgs::LaserScan>("region_point_2", 3);
        scan_pub = node.advertise<sensor_msgs::LaserScan>("filter_scan", 3);
        leg_pub = node.advertise<sensor_msgs::PointCloud>("leg_point", 3);
        center_pose_pub = node.advertise<geometry_msgs::PoseStamped>("center_pose", 3);
        frame = "scan";
    }

    void scanCallback(sensor_msgs::LaserScanPtr scan){
        scan->header.frame_id;//存储文件名称
        laser_filter.filterRack(scan);

        auto &compute_info = laser_filter.getComputeInfo();
        geometry_msgs::PoseStamped center_pose;
        EulerToQuaternion(compute_info.rack_center_pose[2], 0, 0, center_pose.pose.orientation);
        center_pose.pose.position.x = compute_info.rack_center_pose[0];
        center_pose.pose.position.y = compute_info.rack_center_pose[1];
        center_pose.header.frame_id = frame;
        center_pose.header.stamp = ros::Time::now();
        sensor_msgs::PointCloud point_cloud;
        sensor_msgs::PointCloud region_point_cloud;

        for (auto &leg:compute_info.legs) {
            point_cloud.points.emplace_back();
            point_cloud.points.back().x = leg[0];
            point_cloud.points.back().y = leg[1];
        }

        scan_pub.publish(scan);
        scan->header = center_pose.header;

        if (compute_info.regions.size() >= 1) {
            int index = 0;
            for (auto &range:scan->ranges) {
                range = compute_info.regions[0][index];
                index++;
            }
            rectangle_pub.publish(scan);
        }
        if (compute_info.regions.size() >= 2) {
            int index = 0;
            for (auto &range:scan->ranges) {
                range = compute_info.regions[1][index];
                index++;
            }
            rectangle_pub_1.publish(scan);
        }
        if (compute_info.regions.size() >= 3) {
            int index = 0;
            for (auto &range:scan->ranges) {
                range = compute_info.regions[2][index];
                index++;
            }
            rectangle_pub_2.publish(scan);
        }

        point_cloud.header = center_pose.header;
        center_pose_pub.publish(center_pose);
        leg_pub.publish(point_cloud);

//        rectangle_pub.publish(scan);
    }
private:
    void EulerToQuaternion(double yaw, double roll, double pitch, geometry_msgs::Quaternion &quaternion_) {
        double cos_yaw_2 = cos(yaw / 2.0);
        double sin_yaw_2 = sin(yaw / 2.0);
        double cos_roll_2 = cos(roll / 2.0);
        double sin_roll_2 = sin(roll / 2.0);
        double cos_pitch_2 = cos(pitch / 2.0);
        double sin_pitch_2 = sin(pitch / 2.0);
        quaternion_.w = cos_yaw_2 * cos_roll_2 * cos_pitch_2 + sin_yaw_2 * sin_roll_2 * sin_pitch_2;
        quaternion_.x = cos_yaw_2 * sin_roll_2 * cos_pitch_2 + sin_yaw_2 * cos_roll_2 * sin_pitch_2;
        quaternion_.y = cos_yaw_2 * cos_roll_2 * sin_pitch_2 - sin_yaw_2 * sin_roll_2 * cos_pitch_2;
        quaternion_.z = sin_yaw_2 * cos_roll_2 * cos_pitch_2 - cos_yaw_2 * sin_roll_2 * sin_pitch_2;
    }

    ros::NodeHandle node;
    ros::Subscriber scan_sub;
    ros::Publisher rectangle_pub;
    ros::Publisher rectangle_pub_1;
    ros::Publisher rectangle_pub_2;
    ros::Publisher scan_pub;
    ros::Publisher leg_pub;
    ros::Publisher center_pose_pub;
    laser::RackLegsFilter<sensor_msgs::LaserScanPtr> laser_filter;
    std::string frame;

};


#endif //PROJECT_LASER_FILTER_HPP
