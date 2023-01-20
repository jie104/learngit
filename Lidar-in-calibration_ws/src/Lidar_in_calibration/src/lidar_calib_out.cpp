//
// Created by zy on 22-7-26.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>
#include <cmath>
#include <chrono>

#include <boost/circular_buffer.hpp>

using namespace std;

typedef boost::circular_buffer<sensor_msgs::LaserScan> ScanCircularBuffer;

/// 对simi雷达按照点的距离校正，加上误差和距离的关系: range_error = -0.00053527 × range + 0.0309968
// range += range_error
// 输出星秒雷达校准后的点云，看是否和倍加福点云重合
class LidarInternalCalibration {
public:
    LidarInternalCalibration() {
        ros::NodeHandle nh_param("~");
//        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/scan");
        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/simiscan");
        lidar_false_sub_ = nh_.subscribe(lidar_false_topic_, 30, &LidarInternalCalibration::LidarFalseCallback, this);

        // 输出校正之后的LaserScan
        lidar_pub_calibScan_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_calib", 10);
    }
    ~LidarInternalCalibration() {}


    /// 根据之前拟合出的误差，对接受到的每一帧星秒雷达LaserScan进行误差补偿
    void LidarFalseCallback(const sensor_msgs::LaserScanConstPtr &_lidar_false_msg) const {
        sensor_msgs::LaserScan lidar_calibrated = *_lidar_false_msg; // 复制原来的一帧激光，接下来对其每一个激光点补偿距离误差
        for (int i = 0; i < lidar_calibrated.ranges.size(); i++) {
            /// 1、按照距离校准雷达, 对simi雷达按照点的距离校正，加上 error = -0.00053527 × r + 0.0309968
            // 2、simi雷达标第二包数据，error=0.00378883*range + 0.01129387
            // 3、7.27下午采集的数据1，error=-0.00748174*range + 0.05510022
            // 4、error=-0.00748174*range + 0.05510022
//            lidar_calibrated.ranges[i] = lidar_calibrated.ranges[i] + (-0.00703811) * lidar_calibrated.ranges[i] + 0.05354649;

            // 8.3上午采集的数据，500帧平均的误差，补偿上去
            // y = 0.000565052 * x + 0.0249969, 稳定的有0.025m的误差，比倍加福小2.5cm
            lidar_calibrated.ranges[i] = lidar_calibrated.ranges[i] + (0.000565052) * lidar_calibrated.ranges[i] + 0.0249969;
        }
        lidar_pub_calibScan_.publish(lidar_calibrated); // 发布误差校正之后的一帧激光
        cout << "calibrate 1 LaserScan\n";
    }

public:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_false_sub_;
    ros::Publisher lidar_pub_calibScan_; // 发布校正之后的laserScan
    string lidar_false_topic_;
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "lidar_range_calibrated_pub");
    LidarInternalCalibration myLidarInternalCalibration;
    ros::spin();
    return 0;
}

