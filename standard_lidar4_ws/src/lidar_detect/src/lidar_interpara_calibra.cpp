//
// Created by zxj on 2022/9/16.
//


#include <iostream>
#include <ros/ros.h>
#include "lidar_interpara_calibra.hpp"



int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"LIDAR_INSPECT");
    LOG(INFO) << "begin to cal error_range and cal angle_error !!!";
    calibra::LidarInterParaLibra range_ave_error;

    ros::spin();
    return 0;
}