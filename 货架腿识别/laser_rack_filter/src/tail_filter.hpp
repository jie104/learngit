//
// Created by lfc on 18-12-15.
//

#ifndef PROJECT_TAIL_FILTER_HPP
#define PROJECT_TAIL_FILTER_HPP

#define ROS_DEBUG_API

#include <ros/ros.h>
#include <glog/logging.h>
#include "scan_processor.hpp"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include "laser_filter/rack_filter.hpp"
#include "laser_filter/rack_detector.hpp"


class TailFilter {

public:
    TailFilter() {
        scan_sub = node.subscribe("scan", 5, &TailFilter::scanCallback, this);

        scan_pub = node.advertise<sensor_msgs::LaserScan>("filter_scan", 3);
        rack_pub = node.advertise<sensor_msgs::PointCloud>("rack_points", 3);
        region_pub = node.advertise<nav_msgs::Path>("filter_path", 3);

        frame = "scan";

        rack::InstallPara para;
        rack::RackPara rack_para;
        rack_filter.reset(new rack::RackFilter<sensor_msgs::LaserScanPtr>(para));
        rack_filter->computeRackInfo(rack_para);
    }

    //货架腿滤波
    void scanCallback(sensor_msgs::LaserScanPtr scan) {
        scan->header.frame_id;//存储文件名称
//        tail_filter.SetScan(scan);
//        tail_filter.DownSample(1200);
        std::vector<rack::RackInfo_Ptr> rack_infos;
        rack::RackInfo info;

        info.leg_groups.emplace_back();

        info.leg_d = 0.15;  //货架腿直径
        info.leg_groups.back().length = 0.96;
        info.leg_groups.back().width = 0.64;
        rack_infos.emplace_back(new rack::RackInfo());
        *rack_infos.back() = info;
//        info.leg_groups.back().length = 1.20;
//        rack_infos.emplace_back(new rack::RackInfo());
//        *rack_infos.back() = info;
//        info.leg_d = 0.06;
//        info.leg_groups.back().width = 0.65;
//        info.leg_groups.back().length = 1.16;
//        rack_infos.emplace_back(new rack::RackInfo());
//        *rack_infos.back() = info;
//        info.leg_d = 0.05;
//        info.leg_groups.back().width = 0.8;
//        info.leg_groups.back().length = 1.35;
//        info.leg_groups.emplace_back();
//        info.leg_groups.back().length = 1.13;
//        info.leg_groups.back().width = 0.45;
//        rack_infos.emplace_back(new rack::RackInfo());
//        *rack_infos.back() = info;
        rack::RackDetector<sensor_msgs::LaserScanPtr> rack_detector(rack_filter->install_para_,rack_infos);
        rack::RackInfo_Ptr para(new rack::RackInfo);

        std::vector<Eigen::Vector2d> legs;

        sensor_msgs::LaserScanPtr bk_scan(new sensor_msgs::LaserScan);
        *bk_scan = *scan;
        rack_detector.detectRack(bk_scan, para, legs, 0.0);

        std::vector<rack::ClusterPoint_Ptr> rack_clusters;
        rack_filter->computeRackInfo(para);
        rack_filter->filterRack(scan, 0);
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = scan->header;
        for(auto& rack_leg:rack_clusters){
            for(auto& leg:rack_leg->infos){
                point_cloud.points.emplace_back();
                point_cloud.points.back().x = leg.point[0];
                point_cloud.points.back().y = leg.point[1];
            }
        }
//        for(auto& leg:legs){
//            point_cloud.points.emplace_back();
//            point_cloud.points.back().x = leg[0];
//            point_cloud.points.back().y = leg[1];
//        }
        LOG(INFO) << "cluster size:" << rack_clusters.size();
        rack_pub.publish(point_cloud);

//        auto first_point = rack_filter->scan_to_center_tf_.inverse()*Eigen::Vector2d(rack_filter->rectangles_[0].rectangle_->min_x,rack_filter->rectangles_[0].rectangle_->min_y);
//        auto second_point = rack_filter->scan_to_center_tf_.inverse()*Eigen::Vector2d(rack_filter->rectangles_[0].rectangle_->min_x,rack_filter->rectangles_[0].rectangle_->max_y);
//        auto third_point = rack_filter->scan_to_center_tf_.inverse()*Eigen::Vector2d(rack_filter->rectangles_[0].rectangle_->max_x,rack_filter->rectangles_[0].rectangle_->max_y);
//        auto forth_point = rack_filter->scan_to_center_tf_.inverse()*Eigen::Vector2d(rack_filter->rectangles_[0].rectangle_->max_x,rack_filter->rectangles_[0].rectangle_->min_y);
//        nav_msgs::Path path;
//        path.header = scan->header;
//        path.poses.emplace_back();
//        path.poses.back().pose.position.x = first_point[0];
//        path.poses.back().pose.position.y = first_point[1];
//        path.poses.emplace_back();
//        path.poses.back().pose.position.x = second_point[0];
//        path.poses.back().pose.position.y = second_point[1];
//        path.poses.emplace_back();
//        path.poses.back().pose.position.x = third_point[0];
//        path.poses.back().pose.position.y = third_point[1];
//        path.poses.emplace_back();
//        path.poses.back().pose.position.x = forth_point[0];
//        path.poses.back().pose.position.y = forth_point[1];
//        region_pub.publish(path);
//        path.poses.emplace_back();
//        path.poses.back().pose.position.x = first_point[0];
//        path.poses.back().pose.position.y = first_point[1];
//        region_pub.publish(path);

//        filterLowIntenPoints(scan, 1.5);
//        filterScanTraildPoint<sensor_msgs::LaserScanPtr>(scan, 0.8,2,0.03);
//        filterScanTraildPoint<sensor_msgs::LaserScanPtr>(scan, 0.8,1,0.03);
//        filterScanPoint<sensor_msgs::LaserScanPtr>(scan);
//        sensor_msgs::LaserScanPtr filter_scan(new sensor_msgs::LaserScan);
//        *filter_scan = *scan;
//        for (auto &range:filter_scan->ranges) {
//            range = 0;
//        }
//        for (auto &index:indexes) {
//            filter_scan->ranges[index] = scan->ranges[index];
//        }
//        LOG(INFO) << "index size:" << indexes.size();
        scan_pub.publish(scan);

//        rectangle_pub.publish(scan);
    }

    template <class LaserScan_ptr>
    void filterScanTraildPoint(LaserScan_ptr scan,double max_distance,int step = 2,double min_thresh = 0.02) {
        auto& ranges = scan->ranges;
        std::vector<int> indexes;
        double cos_increment = cos(scan->angle_increment * (double)step);
        double theta_thresh=sin((double)scan->angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
        int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance) {
                continue;
            }
            double dist_direction = (ranges[i + step] - ranges[i - step]);

            for (int k = -step; k < step; ++k) {
                double tmp_direction = (ranges[i + k + 1] - ranges[i + k]);
                if (dist_direction * tmp_direction <= 0) {
                    continue;
                }
            }
            double dist_1 = std::sqrt(ranges[i+step] * ranges[i+step] + ranges[i - step] * ranges[i - step] -
                                      2 * ranges[i+step] * ranges[i - step] * cos_increment);

            double range_thresh_1 = ranges[i + step] * theta_thresh + min_thresh;
            if(dist_1 > range_thresh_1) {
                int remove_gap = step - 1;
                for (int j = -remove_gap; j <= remove_gap; ++j) {
                    indexes.push_back(i + j);
                }
            }
        }
        for (auto &index:indexes) {
            ranges[index] = 100;
        }
    }

    template <class LaserScan_ptr>
    void filterLowIntenPoints(LaserScan_ptr scan,double max_distance){
        int scan_size = scan->ranges.size();
        for (int i = 0; i < scan_size; ++i) {
            if (scan->ranges[i] < max_distance) {
                if (scan->intensities[i] < 150) {
                    scan->ranges[i] = 0;
                }
            }
        }
    }

    template <class LaserScan_ptr>
    void filterScanPoint(LaserScan_ptr scan) {
        auto& ranges = scan->ranges;
        double theta_thresh=sin((double)scan->angle_increment)/sin(0.170);//临界值,用于识别断点
        double min_thresh = 0.03f;
        int scan_size = ranges.size() - 1;
        std::vector<int> indexes;
        for (int i = 1; i < scan_size; i++) {
            if (ranges[i] == 100 || ranges[i] == 0) {
                continue;
            }
            float dist_1 = fabs(ranges[i] - ranges[i - 1]);
            float dist_2 = fabs(ranges[i + 1] - ranges[i]);
            float range_thresh_1 = ranges[i - 1] * theta_thresh + min_thresh;
            float range_thresh_2 = ranges[i + 1] * theta_thresh + min_thresh;
            if (dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
                indexes.push_back(i);
            }
        }
        for (auto &index:indexes) {
            ranges[index] = 100;
        }
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

    ros::Publisher scan_pub;
    ros::Publisher rack_pub;
    ros::Publisher region_pub;
    scan::ScanProcessor<sensor_msgs::LaserScanPtr> tail_filter;
    std::string frame;

    std::shared_ptr<rack::RackFilter<sensor_msgs::LaserScanPtr>> rack_filter;
};



#endif //PROJECT_TAIL_FILTER_HPP
