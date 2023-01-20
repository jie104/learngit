#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <glog/logging.h>
#include "K-M_filter.hpp"
#include <memory>



class sub_pub
{
public:

    sub_pub(double max_distance,double min_thresh)
        :max_distance_(max_distance),min_thresh_(min_thresh){

        ROS_INFO("订阅话题");
        sub_=nh_.subscribe<sensor_msgs::LaserScan>("/scan",1000,&sub_pub::filter,this);

        // pub_=nh_.advertise<sensor_msgs::LaserScan>("/scan_pub",1000);

    }

    void filter(const sensor_msgs::LaserScan::ConstPtr& scan ){
        ranges = scan->ranges;
        std::vector<int> indexes;
        const int step = 2;
        double cos_increment = cos(scan->angle_increment * (double)step);
        double theta_thresh=sin((double)scan->angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
        int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance_) {
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

            double range_thresh_1 = ranges[i] * theta_thresh + min_thresh_;
            double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh_;
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
   
    scan_.ranges=ranges;
    scan_.angle_min=scan->angle_min;
    scan_.angle_max=scan->angle_max; 
    scan_.angle_increment=scan->angle_increment;
    scan_.range_max=scan->range_max;
    scan_.range_min=scan->range_min;
    scan_.header.frame_id="scan";
    pub_.publish(scan_);

}


private:
    double max_distance_;    //TODO
    double min_thresh_;  //TODO
    std::vector<float> ranges;  //测距距离

    //初始节点句柄
    ros::NodeHandle nh_;

    //订阅
    ros::Subscriber sub_;

    //发布
    ros::Publisher pub_=nh_.advertise<sensor_msgs::LaserScan>("/scan_pub",1000);

    sensor_msgs::LaserScan scan_;
};



static ros::Publisher pub;  //链接性为内部

void filter(const sensor_msgs::LaserScan::ConstPtr& scan){

/*****************************数据处理模块****************************************/
    const int step=2;
    double angle_min=scan->angle_min;
    double angle_increment=scan->angle_increment;
    std::vector<float> ranges=scan->ranges;
    std::vector<std::shared_ptr<KM::ScanPoint>> clusters;
    KM::getClusterFromScan(scan,clusters);
    // LOG(INFO) << "begin to filter!!!";
    int j=0;
    for (auto& cluster:clusters){
        int i=(cluster->points[0].angle-angle_min)/angle_increment;
        // if (i>=2 /*&& ranges[i] <3*/){
            // j++;
            // LOG(INFO) << "第" << j << "个  " << "i: " << i ;
        // for (int j=i-step;j<= i+step;j++){
        //     ranges[j]=100;
        // }
        // }
        for (int j=0;j<cluster->points.size();j++){
            ranges[i+j]=100;
        }

    }

/*****************************************************************************/

    sensor_msgs::LaserScan scan1;
    // scan1.angle_increment=scan->angle_increment;
    // LOG(INFO) << "angle_increment:" << scan1.angle_increment;
    // scan1.angle_max=scan->angle_max;
    // scan1.angle_min=scan->angle_min;
    // scan1.range_max=scan->range_max;
    // scan1.range_min=scan->range_min;
    scan1=*scan;
    scan1.ranges=ranges;
    scan1.header.frame_id="scan";

    pub.publish(scan1);
}





int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"lidar");
    ros::NodeHandle nh;

    pub=nh.advertise<sensor_msgs::LaserScan>("/scan_filter",1000);
    ros::Subscriber sub=nh.subscribe<sensor_msgs::LaserScan>("/scan",1000,filter);
    // sub_pub Sub_pub(3,0.020);

    
    ros::spin();
    return 0;
}
