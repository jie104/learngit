//
// Created by lfc on 2021/12/18.
//

#ifndef RACK_DETECTOR_RACK_DETECTOR_OPERATOR_HPP
#define RACK_DETECTOR_RACK_DETECTOR_OPERATOR_HPP
#include <vector>
#include <ros/ros.h>
#include <glog/logging.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "rack_detection/rack_detecter.hpp"
#include "cluster_rack_detection/cluster_detector.hpp"
#include "cluster_rack_detection/triangle_detecor.hpp"
#include "cluster_rack_detection/rectangle_detector.hpp"
namespace rack_detection {
class RackDetectorOperator {
 public:
    RackDetectorOperator() {
//        rack_infos.emplace_back();
//        rack_infos.back().leg_d = 0.05f;
//        rack_infos.back().leg_groups.emplace_back();
//        rack_infos.back().leg_groups.back().length = 1.44;
//        rack_infos.back().leg_groups.back().width = 0.95;
        rack_infos.emplace_back();
        rack_infos.back().leg_d = 0.1f;
        rack_infos.back().leg_groups.emplace_back();
        rack_infos.back().leg_groups.back().length = 0.7;
        rack_infos.back().leg_groups.back().width = 0.7;
        for (auto& rack : rack_infos) {
            detector.addRackInfo(rack);
        }
        scan_sub = node.subscribe("scan", 5, &RackDetectorOperator::onScanCallback, this);
        point_pub = node.advertise<sensor_msgs::PointCloud>("clusters", 5);
        pose_pub = node.advertise<geometry_msgs::PoseArray>("poses", 5);
        scan_pub = node.advertise<sensor_msgs::LaserScan>("filtered_scan", 5);
    }

    //货架腿识别
    void onScanCallback(sensor_msgs::LaserScan scan){
        std::vector<RackInfo> racks;
        std::vector<Eigen::Vector3d> center_poses;
        std::shared_ptr<sensor_msgs::LaserScan> scan_ptr(new sensor_msgs::LaserScan);
        *scan_ptr = scan;

        rack_detection::ClusterDetector cluster_detector;
        std::vector<ClusterPoint_Ptr> clusters;
        cluster_detector.extractClusters(scan_ptr, clusters);   //找出货架腿聚类点，存放在clusters
        rack_detection::TriangleDetecor triangle_detector;
        std::vector<TriangleInfo_Ptr> triangles;
        triangle_detector.detectAngle(clusters, triangles); //找出所有货架腿位姿及中心，存放在triangles中
        rack_detection::RectangleDetector rectangle_detector;
        std::vector<rack_detection::RectangleInfo_Ptr> rectangles;
        rectangle_detector.computeRectangle(triangles, rectangles); //将重复货架归一化，提高估计精度，存放在rectangles中
        rectangle_detector.filterRectangleBySize(rectangles, 0.7, 0.7, 0.03 + 0.05);        //同过货架实际尺寸过滤货架

        sensor_msgs::PointCloud cloud;
        cloud.header = scan.header;
        for(auto& cluster:clusters) {
            cloud.points.emplace_back();
            cloud.points.back().x = cluster->mean[0];
            cloud.points.back().y = cluster->mean[1];
        }
        point_pub.publish(cloud);

        geometry_msgs::PoseArray pose_array;
        pose_array.header = scan.header;
        for (auto& rectangle : rectangles) {
            pose_array.poses.emplace_back();
            pose_array.poses.back().position.x = rectangle->mean[0];
            pose_array.poses.back().position.y = rectangle->mean[1];
            EulerToQuaternion(atan2(rectangle->direction[1], rectangle->direction[0]), 0, 0,
                              pose_array.poses.back().orientation);
        }
        LOG(INFO) << "rectangle size:" << rectangles.size() <<"triangle size:"<< triangles.size();
        pose_pub.publish(pose_array);
        //        detector.detectRack(scan_ptr, racks, center_poses);
////        LOG(INFO) << "rack size:" << racks.size();
//        if (center_poses.size()) {
//            geometry_msgs::PoseStamped pose;
//            pose.header = scan.header;
//            pose.pose.position.x = center_poses[0][0];
//            pose.pose.position.y = center_poses[0][1];
//            Eigen::Vector2d mean = center_poses[0].head<2>();
//            auto direction = Eigen::Vector2d(cos(center_poses[0][2]), sin(center_poses[0][2]));
//            Eigen::Vector2d verticle = Eigen::Vector2d(-direction[1], direction[0]);
//            if (mean.dot(verticle) < 0.0) {
//                verticle = -verticle;
//            }
//            auto real_direction = mean.dot(verticle) > mean.dot(direction) ? verticle : direction;
//            LOG(INFO) << "real:" << atan2(real_direction[1], real_direction[0]);
//            EulerToQuaternion(atan2(real_direction[1],real_direction[0]), 0, 0, pose.pose.orientation);
//            point_pub.publish(pose);
//        }
//        scan_pub.publish(*scan_ptr);
    }

 private:
    template <class Quaternion>
    void EulerToQuaternion(double yaw, double roll, double pitch, Quaternion &quaternion_) {
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

    std::vector<RackInfo> rack_infos;
    RackDetecter<sensor_msgs::LaserScan> detector;
    ros::NodeHandle node;
    ros::Subscriber scan_sub;
    ros::Publisher pose_pub;
    ros::Publisher point_pub;
    ros::Publisher scan_pub;

};
}  // namespace rack_detection

#endif  // RACK_DETECTOR_RACK_DETECTOR_OPERATOR_HPP
