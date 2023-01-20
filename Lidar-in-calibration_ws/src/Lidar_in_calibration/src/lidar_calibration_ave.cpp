//
// Created by zy on 22-7-26.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

#include <boost/circular_buffer.hpp>

using namespace std;
//using namespace chrono;

static int ave_true_num = 0;
static int ave_false_num = 0;
Eigen::VectorXd polyfit(vector<pair<double, double>> &in_point, int n);

class LidarInternalCalibration {
public:
    LidarInternalCalibration() {
        ros::NodeHandle nh_param("~");
        nh_param.param<string>("lidar_true_topic", lidar_true_topic_, "/r2000_node/scan");

        // 新的oradar雷达的误差很小，0.001m,和倍加福雷达点云重合，不用标定了
//        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/scan"); // oradar的topic

        // 星秒雷达，使用静止测量的数据，计算平均误差，拟合误差曲线。
        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/simiscan"); // simi雷达

//        // 误差补偿之后的星秒雷达
//        nh_param.param<string>("lidar_false_topic", lidar_false_topic_, "/scan_calib"); // simi雷达

        lidar_true_sub_ = nh_.subscribe(lidar_true_topic_, 10, &LidarInternalCalibration::LidarTrueCallback, this);
        lidar_false_sub_ = nh_.subscribe(lidar_false_topic_, 30, &LidarInternalCalibration::LidarFalseCallback, this);

        pcl_pub_calibPoint_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_false_point_calib", 10);

    }
    ~LidarInternalCalibration() {}

    // r2000的真值，累加平均
    void LidarTrueCallback(const sensor_msgs::LaserScanConstPtr &_lidar_true_msg) {
        ave_true_num++;
        current_laserscan_true_msg_ = _lidar_true_msg; // 类成员变量 指针ConstPtr，得到激光雷达的信息

        if (ave_true_num == 1) {
            ave_true_scan_ = *_lidar_true_msg;
        } else { // 平均range, 求和，统计数量。
            for (int i = 0; i < ave_true_scan_.ranges.size(); i++) {
                ave_true_scan_.ranges[i] += _lidar_true_msg->ranges[i];
            }
        }

    }

    /*
     * aobi雷达intensity为0的点忽略。 奥比雷达range范围range_min: 0.009999999776482582;  range_max: 40.0    [0.01, 40]
     * 角度范围： [-2.356194496154785,  2.356194496154785]    angle_increment: 0.004090615548193455
     * 倍加福雷达，range = 1048.574951171875 这些点要过滤掉， range_min: 0.0; range_max: 30.0    过滤掉range范围以外的点，不计算误差 [0, 30]
     * 角度范围： [-3.1415927410125732, 3.1415927410125732]   angle_increment: 0.001745329238474369
     * */

    /// 计算低精度雷达这一帧每一个激光点的range误差（和相同角度的/r2000的range比较）
    void LidarFalseCallback(const sensor_msgs::LaserScanConstPtr &_lidar_false_msg) {
        ave_false_num++;
        current_laserscan_false_msg_ = _lidar_false_msg;

        if (ave_false_num == 1) {
            ave_false_scan_ = *_lidar_false_msg;
        } else {
            for (int i = 0; i < ave_false_scan_.ranges.size(); i++) {
                ave_false_scan_.ranges[i] += _lidar_false_msg->ranges[i];
                ave_false_scan_.intensities[i] += _lidar_false_msg->intensities[i];
            }
        }

//        if (ave_false_num != 200) {
        if (ave_false_num != 500) {
            return; // 平均了500帧之后，才计算误差.
        }
        for (int i = 0; i < ave_false_scan_.ranges.size(); i++) {
            ave_false_scan_.ranges[i] /= 500.0f;
            ave_false_scan_.intensities[i] /= 500.0f; // 500帧平均
        }

        for (int i = 0; i < ave_true_scan_.ranges.size(); i++) {
            ave_true_scan_.ranges[i] /= (float)ave_true_num;
        }

        int length = _lidar_false_msg->ranges.size(); // 1152

        double angle_start = _lidar_false_msg->angle_min; // -2.35619
        double angle_end = _lidar_false_msg->angle_max; // 2.35619
        double angle_incre = _lidar_false_msg->angle_increment; // 0.00409062

        // 在sensor_msgs/LaserScan 中，所有数据都是float32
        vector<pair<double, double>> pointCloudFalseRange(length); // 每个点，pair<角度，距离>
        vector<pair<double, double>> pointCloudFalseIntensity(length); // 每个点，pair<角度，强度>

        for (int i = 0; i < length; i++) {
            double r = ave_false_scan_.ranges[i];
            double theta = angle_start + angle_incre * i;
            pointCloudFalseRange[i] = {theta, r};
            double intensity = ave_false_scan_.intensities[i];
            pointCloudFalseIntensity[i] = {theta, intensity};
        }

        vector<pair<double, double>> pointCloudTrueRange;
        pointCloudTrueRange.reserve(current_laserscan_true_msg_->ranges.size()); // 预留空间，保存过滤后的r2000的激光点
        // 不用缓存区最新的一帧，用累加平均之后的true_scan
        const sensor_msgs::LaserScan &_lidar_true_msg = ave_true_scan_; // /r2000的真值
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

        static int index_front = 0;
        static int index_back = 0;
        /// 低精度雷达的range, intensity, error, getPointError的size相等，下标是对应的
        vector<pair<double, double>> pointCloudError(length); // pair<角度，距离误差(r2000-奥比)>
        vector<bool> getPointError(length, false); // 记录是否得到当前角度的误差值
        // 忽略开头和结束的几个激光点
        for (int i = 5; i < length - 5; i++) {
            double theta = pointCloudFalseRange[i].first; // 低精度雷达的theta角度
            double range_false = pointCloudFalseRange[i].second;
            // 小于0.3m的太近的点都不计算误差，两个雷达扫到的环境不一样
            if (range_false > 8 || range_false < 0.3) {
                continue;
            }
            // 找到最近的两个高精度雷达角度
            for (int j = index_back; j < pointCloudTrueRange.size(); j++) {
                if (pointCloudTrueRange[j].first >= theta) {
                    index_back = j;
                    index_front = j - 1;
                    break;
                }
            }

            if (abs(pointCloudTrueRange[index_front].second - pointCloudTrueRange[index_back].second) > 0.1) { // 设置的阈值0.1
                continue; // 不进行插值
            }

            double range_true = pointCloudTrueRange[index_front].second +
                                (pointCloudTrueRange[index_back].second - pointCloudTrueRange[index_front].second)
                                * (theta - pointCloudTrueRange[index_front].first)
                                / (pointCloudTrueRange[index_back].first - pointCloudTrueRange[index_front].first);
            double err = range_true - range_false;

            if (abs(err) > 0.1) { // 自己设置的阈值, 0.1m, 0.05
                continue; // 误差过大，可能是环境不同导致的，不考虑
            }
            getPointError[i] = true; // 标志
            pointCloudError[i] = {theta, err};
        }
        index_front = 0;
        index_back = 0; // 处理完一帧数据之后，下标置0

//        vector<pair<int, double>> pointCalib; // pair<强度， 误差>
        vector<pair<double, double>> pointCalib2; // pair<range, 误差>
        pcl::PointCloud<pcl::PointXYZI> calibPoints; // 显示点云，调整过滤阈值
        pcl::PointXYZI p;
        p.z = 0.0;
        for (int i = 5; i < length - 5; i++) {
            if (!getPointError[i]) {
                continue;
            }
//            pointCalib.emplace_back(pointCloudFalseIntensity[i].second, pointCloudError[i].second);
//            cout << pointCloudFalseIntensity[i].second << "," << pointCloudError[i].second << endl;

            pointCalib2.emplace_back(pointCloudFalseRange[i].second, pointCloudError[i].second);
//            cout << pointCloudFalseRange[i].second << "," << pointCloudError[i].second << endl;
            p.x = pointCloudFalseRange[i].second * cos(pointCloudFalseRange[i].first);
            p.y = pointCloudFalseRange[i].second * sin(pointCloudFalseRange[i].first);
            p.intensity = pointCloudFalseIntensity[i].second;
            calibPoints.push_back(p);
        }

        sensor_msgs::PointCloud2 calibPoint_msg;
        if (pub_false_pointcloud_) {
            pcl::toROSMsg(calibPoints, calibPoint_msg);
            calibPoint_msg.header.frame_id = "scan";
            pcl_pub_calibPoint_.publish(calibPoint_msg);
        }


        // 输出到文本文件中
//        cout << "处理完200帧的平均误差！！！！" << endl;
        cout << "处理完500帧的平均误差！！！！" << endl;

        // 拟合误差（拟合直线）
        int n = 1; // 1次多项式（直线y = kx + b）
        Eigen::VectorXd polt1 = polyfit(pointCalib2, n);
        cout << "polt1 : " << polt1 << endl;

        cout << " write file !!" << endl;

        /// 把每个角度的误差和强度对应，输出（强度，误差）
//        ofstream write;
//        for (int i = 0; i < pointCalib.size(); i++) {
//            write.open("/home/zy/雷达内参标定/bag/new-3/1-ave500-simi/calib_in_err.csv", ios::app);	//用ios::app不会覆盖文件内容
//            write << pointCalib[i].first << "," << pointCalib[i].second << endl;
//            write.close();
//        }
//
        /// 把每个角度的误差和距离对应，输出（range，误差）
        ofstream write2;
        for (int i = 0; i < pointCalib2.size(); i++) {
            write2.open("/home/zy/雷达内参标定/bag/8-3-4lidar/1-ave500-simi/calib_range_err_calib.csv", ios::app);	//用ios::app不会覆盖文件内容
            write2 << pointCalib2[i].first << "," << pointCalib2[i].second << endl;
            write2.close();
        }
        cout << "write OK!" << endl;
    }

public:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_true_sub_;
    ros::Subscriber lidar_false_sub_;
    ros::Publisher pcl_pub_calibPoint_;

    string lidar_true_topic_;
    string lidar_false_topic_;

    bool pub_false_pointcloud_ = true; // 发布需要校正的点云

    // 指向接收到的两种雷达msg，得到两种雷达的信息
    sensor_msgs::LaserScan::ConstPtr current_laserscan_false_msg_;
    sensor_msgs::LaserScan::ConstPtr current_laserscan_true_msg_;

    // 将两种激光平均
    sensor_msgs::LaserScan ave_false_scan_;
    sensor_msgs::LaserScan ave_true_scan_;
};

/**
 *
 * @param in_point
 * @param n  多项式阶数
 * @return
 */
Eigen::VectorXd polyfit(vector<pair<double, double>> &in_point, int n) {
    int size = in_point.size();
    int x_num = n + 1; // n阶多项式，n + 1个未知数
    cout << "size = " << size << ", x_num = " <<x_num << endl;
    Eigen::MatrixXd U( size, n + 1);
    Eigen::VectorXd y(size);
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < x_num; ++j) {
            U(i, j) = pow(in_point[i].first, j); // 1, x, x^2, x^3, x^4,...
            cout << U(i, j) << ", ";
        }
    }

    cout << endl;
    for (int i = 0; i < size; i++) {
        y(i) = in_point[i].second;
        cout << y(i) << ", ";
    }
    cout << endl;
    // 方程数量大于未知数个数，超定方程求解  Y = U * K

    auto Q = U.householderQr();
    auto result = Q.solve(y);
    cout << "result = " << result.transpose() << endl;// result =   0.0270659, 1.73292e-05
    // result =    0.0281384 -0.000168315
    // result =   0.0265385 0.000488132
    // result =   0.0249969 0.000565052    y = 0.000565052 * x + 0.0249969, 稳定的有0.025m的误差，比倍加福小2.5cm

    // [0.00257065 0.02021297] python拟合的结果
    // result =   0.020213 0.00257065，C++ Eigen计算结果，相同。 range_error=0.00257065*range + 0.020213

    // [-0.00199433  0.03649678] python
    // result =   0.0364968 -0.00199433

    // 校准之后的误差 result = -0.000487684   0.00037496
    return result;
}


// 500帧取均值，计算平均误差
int main (int argc, char **argv) {
    ros::init(argc, argv, "lidar_internal_calibration_average");
    LidarInternalCalibration myLidarInternalCalibration;
    ros::spin();
    return 0;
}

