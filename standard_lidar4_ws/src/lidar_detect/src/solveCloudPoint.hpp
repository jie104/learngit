#ifndef _SLOVE_CLOUD_POINT
#define _SLOVE_CLOUD_POINT


#include <ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include <map>
#include <cmath>
#include <vector>
#include <glog/logging.h>
#include <string>
#include <memory>
#include <fstream>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "base_data_type.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>




namespace scan{

class LidarPointProcessor{
public:
    static void filter(const base_data::LaserScan_ptr& scan,std::vector<float>& ranges,double max_distance, const int min_thresh=0.02){
        ranges = scan->ranges;
        std::vector<int> indexes;
        const int step = 2;
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
            double dist_1 = std::sqrt(ranges[i] * ranges[i] + ranges[i - step] * ranges[i - step] -
                                    2 * ranges[i] * ranges[i - step] * cos_increment);
            double dist_2 = std::sqrt(ranges[i] * ranges[i] + ranges[i + step] * ranges[i + step] -
                                    2 * ranges[i] * ranges[i + step] * cos_increment);
            // double dist_1=ranges[i]*cos((double)scan->angle_increment * (double)step);
            // double dist_2=ranges[i+step]*cos((double)scan->angle_increment * (double)step);

            double range_thresh_1 = ranges[i] * theta_thresh + min_thresh;
            double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh;
            // double range_thresh_1 = ranges[i-step] + min_thresh_;
            // double range_thresh_2 = ranges[i] + min_thresh_;
            // std::cout << "dist_1: " << dist_1 << ", range_thresh_1: "<< range_thresh_1 << std::endl;
            // std::cout << "dist_2: " << dist_2 << ", range_thresh_2: "<< range_thresh_2 << std::endl;
            // std::cout << std::endl;
            if(dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
                for (int j = -step; j <= step; ++j) {
                    indexes.push_back(i + j);
                }
            }

        }
        for (auto &index:indexes) {
            ranges[index] = 100;
        }
   

    }


    //利用回归方程拟合直线,计算倾斜角
    static double calAngle(std::vector<Eigen::Vector2d>& points) {
        double sum_1=0,sum_2=0;
        double mean_x=0,mean_y=0;
        int points_size=points.size();
        std::mutex mute;
        

        for (int i=0;i<points_size;i++){
            sum_1+=points[i][0];
            sum_2+=points[i][1];
        }
        mean_x=sum_1/points_size;
        mean_y=sum_2/points_size;

        sum_1=0;
        sum_2=0;
        for (int j=0;j<points_size;j++){
            sum_1+=(points[j][0]-mean_x)*(points[j][1]-mean_y);
            sum_2+=(points[j][0]-mean_x)*(points[j][0]-mean_x);
        }

        // LOG(INFO) << "SUM_1: " << sum_1;
        // LOG(INFO) << "SUM_2: " << sum_2;

        double theta=atan2(sum_1,sum_2);
        // LOG(INFO) << "theta: " << theta;
        return theta;
        // double tan_theta=sum_1/sum_2;


    }



};








}


#endif