//
// Created by zy on 22-7-20.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

#include <boost/circular_buffer.hpp>
#include <mutex>

using namespace std;

typedef boost::circular_buffer<sensor_msgs::LaserScan> ScanCircularBuffer;

class LidarInternalCalibration {
public:
    LidarInternalCalibration() {
        ros::NodeHandle nh_param("~");
        nh_param.param<string>("lidar_true_topic", lidar_true_topic_, "/r2000_node/scan");
//        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/scan");
        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/simiscan");

        lidar_true_sub_ = nh_.subscribe(lidar_true_topic_, 10, &LidarInternalCalibration::LidarTrueCallback, this);
        lidar_false_sub_ = nh_.subscribe(lidar_false_topic_, 30, &LidarInternalCalibration::LidarFalseCallback, this);

        pcl_pub_calibPoint_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_false_point_calib", 10);

        scanTrueCircularBuffer_ = ScanCircularBuffer(80); // buffer容量, 雷达帧率约20Hz
        scanTrueCircularBuffer_.clear();
    }
    ~LidarInternalCalibration() {}

    /// r2000的真值，放入环形缓冲区的队头
    void LidarTrueCallback(const sensor_msgs::LaserScanConstPtr &_lidar_true_msg) {
        current_laserscan_true_msg_ = _lidar_true_msg; // 类成员变量 得到激光雷达的信息
        unique_lock<mutex> lock1(mutex_cirBuffer);
        scanTrueCircularBuffer_.push_front(*_lidar_true_msg);
        lock1.unlock();
    }

    /*
     * aobi雷达intensity为0的点忽略。 奥比雷达range范围range_min: 0.009999999776482582;  range_max: 40.0    [0.01, 40]
     * 角度范围： [-2.356194496154785,  2.356194496154785]    angle_increment: 0.004090615548193455
     *
     * 倍加福雷达，range = 1048.574951171875 这些点要过滤掉， range_min: 0.0; range_max: 30.0    过滤掉range范围[0, 30]以外的点
     * 角度范围： [-3.1415927410125732, 3.1415927410125732]   angle_increment: 0.001745329238474369
     * */

    /// 计算低精度雷达这一帧每一个激光点的range误差（和相同角度的/r2000的range比较）
    void LidarFalseCallback(const sensor_msgs::LaserScanConstPtr &_lidar_false_msg) {
        current_laserscan_false_msg_ = _lidar_false_msg;

        int length = _lidar_false_msg->ranges.size(); // 1152

        double angle_start = _lidar_false_msg->angle_min; // -2.35619
        double angle_end = _lidar_false_msg->angle_max; // 2.35619
        double angle_incre = _lidar_false_msg->angle_increment; // 0.00409062

        vector<pair<double, double>> pointCloudFalseRange(length); // 每个点，pair<角度，距离>
        vector<pair<double, int>> pointCloudFalseIntensity(length); // 每个点，pair<角度，强度>

        for (int i = 0; i < length; i++) {
            double r = _lidar_false_msg->ranges[i];
            double theta = angle_start + angle_incre * i;
            pointCloudFalseRange[i] = {theta, r};
            int intensity = _lidar_false_msg->intensities[i];
            pointCloudFalseIntensity[i] = {theta, intensity};
        }

        vector<pair<double, double>> pointCloudTrueRange;
        pointCloudTrueRange.reserve(current_laserscan_true_msg_->ranges.size()); // 预留空间，保存过滤后的r2000的激光点
        if (scanTrueCircularBuffer_.size() > 1) {
            unique_lock<mutex> lock1(mutex_cirBuffer);
            const sensor_msgs::LaserScan &_lidar_true_msg = scanTrueCircularBuffer_[0]; // /r2000的真值
            lock1.unlock();
            double theta_true_start = _lidar_true_msg.angle_min;
            double theta_true_incre = _lidar_true_msg.angle_increment;
            for (int i = 0; i < _lidar_true_msg.ranges.size(); i++) {
                double r = _lidar_true_msg.ranges[i];
                double theta = theta_true_start + theta_true_incre * i;
                // r2000点云过滤,角度，距离
                if (theta >= angle_start && theta <= angle_end && r > _lidar_true_msg.range_min && r < _lidar_true_msg.range_max) {
                    pointCloudTrueRange.emplace_back(theta, r);
                }
            }
        } else {
            cout << "wait...\n";
            return; // 没有读到真值r2000的激光scan
        }

        static int index_front = 0;
        static int index_back = 0;
        /// 低精度雷达的range, intensity, error, getPointError的size相等，下标是对应的
        vector<pair<double, double>> pointCloudError(length); // pair<角度，距离误差(r2000-奥比)>
        vector<bool> getPointError(length, false); // 记录是否得到当前角度的误差值
        // i 从5开始，忽略开头和结束的几个激光点
        for (int i = 5; i < length - 5; i++) {
            double theta = pointCloudFalseRange[i].first; // 低精度雷达的theta角度
            double range_false = pointCloudFalseRange[i].second;

            // range范围之外的点，不计算误差
//            if (!(range_false > current_laserscan_false_msg_->range_min && range_false < current_laserscan_false_msg_->range_max)) {
//                continue;
//            }
            if (range_false > 6 || range_false < 0.3) { // 只考虑一定范围内的激光点，雷达原点对齐 安装在不同高度。认为两个雷达扫到的环境是相同的
                continue;
            }

            // 两种雷达角度不是完全对应的，找到与当前低精度雷达角度最近的两个高精度雷达角度
            for (int j = index_back; j < pointCloudTrueRange.size(); j++) {
                if (pointCloudTrueRange[j].first >= theta) {
                    index_back = j;
                    index_front = j - 1;
                    break;
                }
            }

//            if (abs(pointCloudTrueRange[index_front].second - pointCloudTrueRange[index_back].second) > 0.1) { /// 设置的阈值0.1
//                continue; // 不进行插值
//            }

            // 插值得到当前角度下，倍加福雷达的距离
            double range_true = pointCloudTrueRange[index_front].second +
                    (pointCloudTrueRange[index_back].second - pointCloudTrueRange[index_front].second)
                    * (theta - pointCloudTrueRange[index_front].first)
                    / (pointCloudTrueRange[index_back].first - pointCloudTrueRange[index_front].first);
            double err = range_true - range_false; // 距离误差 = 高精度雷达测量值 - 需要标定的低精度雷达的测量值

            if (abs(err) > 0.1) { // 自己设置的阈值, 0.1m, 0.05
                continue; // 误差过大，可能是环境不同导致的，不属于需要校准的误差
            }
            getPointError[i] = true; // 标志这个点是对应点，计算了误差
            pointCloudError[i] = {theta, err};
        }
        index_front = 0;
        index_back = 0; // 处理完一帧数据之后，下标置0

        vector<pair<int, double>> pointCalib; // pair<强度， 误差>
        vector<pair<double, double>> pointCalib2; // pair<range, 误差>
        pcl::PointCloud<pcl::PointXYZI> calibPoints; // 显示点云，调整过滤阈值
        pcl::PointXYZI p;
        p.z = 0.0;
        for (int i = 5; i < length - 5; i++) {
            if (!getPointError[i]) {
                continue;
            }
            pointCalib.emplace_back(pointCloudFalseIntensity[i].second, pointCloudError[i].second);

            pointCalib2.emplace_back(pointCloudFalseRange[i].second, pointCloudError[i].second);
            cout << pointCloudFalseRange[i].second << "," << pointCloudError[i].second << endl;
            p.x = pointCloudFalseRange[i].second * cos(pointCloudFalseRange[i].first);
            p.y = pointCloudFalseRange[i].second * sin(pointCloudFalseRange[i].first);
            p.intensity = pointCloudFalseIntensity[i].second;
            calibPoints.push_back(p);
        }

        sensor_msgs::PointCloud2 calibPoint_msg;
        if (pub_false_pointcloud_) {
            pcl::toROSMsg(calibPoints, calibPoint_msg);
            calibPoint_msg.header.frame_id = "scan"; // 固定同一个坐标系
            pcl_pub_calibPoint_.publish(calibPoint_msg);
        }


        // 误差输出到文本文件中
        cout << "处理完一帧！！！！" << endl;

        cout << " write file !!" << endl;

        /// 把每个角度的误差和强度对应，输出（强度，误差）
//        ofstream write;
//        for (int i = 0; i < pointCalib.size(); i++) {
//            write.open("/home/zy/雷达内参标定/bag/3/r2000-simi-all/calib_in_err.csv", ios::app);	//用ios::app不会覆盖文件内容
//            write << pointCalib[i].first << "," << pointCalib[i].second << endl;
//            write.close();
//        }
//
//        /// 把每个角度的误差和距离对应，输出（range，误差）
//        ofstream write2;
//        for (int i = 0; i < pointCalib2.size(); i++) {
//            write2.open("/home/zy/雷达内参标定/bag/3/r2000-simi-all/calib_range_err.csv", ios::app);	//用ios::app不会覆盖文件内容
//            write2 << pointCalib2[i].first << "," << pointCalib2[i].second << endl;
//            write2.close();
//        }

        cout << "write OK!" << endl;
    }
public:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_true_sub_;
    ros::Subscriber lidar_false_sub_;

    ros::Publisher pcl_pub_calibPoint_;

    ScanCircularBuffer scanTrueCircularBuffer_; // r2000的频率更高，放到缓存区的队头

    string lidar_true_topic_;
    string lidar_false_topic_;

    bool pub_false_pointcloud_ = true; // 发布需要校正的点云

    std::mutex mutex_cirBuffer;
    // 指向接收到的两种雷达msg，得到两种雷达的信息
    sensor_msgs::LaserScan::ConstPtr current_laserscan_false_msg_;
    sensor_msgs::LaserScan::ConstPtr current_laserscan_true_msg_;
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "lidar_internal_calibration_allPoints");
    LidarInternalCalibration myLidarInternalCalibration;
    ros::spin();
    return 0;
}

