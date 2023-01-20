//
// Created by lfc on 19-2-15.
//

#ifndef PROJECT_CHARGING_STATION_WRAP_HPP
#define PROJECT_CHARGING_STATION_WRAP_HPP

#include <ros/ros.h>
#include <glog/logging.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include "charging_station_detector.hpp"

class ChargingStationWrap {
public:
    ChargingStationWrap(){
        scan_sub = node.subscribe("scan", 5, &ChargingStationWrap::scanCallback, this);
        pcl_pub = node.advertise<sensor_msgs::PointCloud>("chargin_points", 3);
        trail_pub = node.advertise<sensor_msgs::PointCloud>("trail_points", 3);
        pose_pub = node.advertise<geometry_msgs::PoseStamped>("charging_pose", 3);
    }


    void scanCallback(sensor_msgs::LaserScanPtr scan){

        sensor_msgs::PointCloud pcl;
        sensor_msgs::PointCloud trail;
        geometry_msgs::PoseStamped station_pose;
        pcl.header = scan->header;
        trail.header = scan->header;
        station_pose.header = scan->header;
//        LOG(INFO) << "begin to detect!!!!!!!!!!!!!!";
        scan->header.frame_id;//存储文件名称
        std::vector<detector::Point> points;
        std::vector<detector::ChargingStationDetector<sensor_msgs::LaserScanPtr>::ContinousPoints> segments;
        std::vector<detector::ChargingStationDetector<sensor_msgs::LaserScanPtr>::Corner> corners,charging_points;
        detector::ChargingStationInfo station_info;
        charging_detector.splitScan(scan->ranges,scan->angle_increment, segments);
        charging_detector.convertTrailingToPoint(scan, segments, corners);
        for (auto &corner:corners) {
            trail.points.emplace_back();
            trail.points.back().x = corner.point.x;
            trail.points.back().y = corner.point.y;
        }

        std::vector<detector::ChargingStationDetectInfo> detect_infos;
        if (charging_detector.findChargingStation(scan, segments,station_info, detect_infos)) {
            for (auto &detect_info:detect_infos) {
                pcl.points.emplace_back();
                pcl.points.back().x = detect_info.first_top.x;
                pcl.points.back().y = detect_info.first_top.y;
                pcl.points.emplace_back();
                pcl.points.back().x = detect_info.second_top.x;
                pcl.points.back().y = detect_info.second_top.y;
                pcl.points.emplace_back();
                pcl.points.back().x = detect_info.center_point.x;
                pcl.points.back().y = detect_info.center_point.y;
            }
            auto &detect_info = detect_infos.back();
            EulerToQuaternion(detect_info.direction, 0, 0, station_pose.pose.orientation);
            station_pose.pose.position.x = detect_info.center_point.x;
            station_pose.pose.position.y = detect_info.center_point.y;
            pose_pub.publish(station_pose);
        }else{
            LOG(INFO) << "cannot find!";
        }
//        for (auto &segment:segments) {
//            points.clear();
//            charging_detector.buildPoints(scan, segment, points);
//            corners.clear();
//            LOG(INFO) << "point size:" << points.size();
//            if (!points.empty()) {
//                charging_detector.extractCorners(points, corners);
//                charging_detector.findChargingStation(points, corners, station_info);
//                LOG(INFO) << "corner size:" << corners.size();
//                for (auto &corner:corners) {
//                    pcl.points.emplace_back();
//                    pcl.points.back().x = corner.point.x;
//                    pcl.points.back().y = corner.point.y;
//                }
//            }
//
//        }
        pcl_pub.publish(pcl);
        trail_pub.publish(trail);

//        rectangle_pub.publish(scan);
    }

    virtual ~ChargingStationWrap(){

    }

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
private:
    detector::ChargingStationDetector<sensor_msgs::LaserScanPtr> charging_detector;
    ros::NodeHandle node;
    ros::Subscriber scan_sub;
    ros::Publisher pcl_pub;
    ros::Publisher trail_pub;
    ros::Publisher pose_pub;
};


#endif //PROJECT_CHARGING_STATION_WRAP_HPP
