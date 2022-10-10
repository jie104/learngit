//
// Created by zxj on 2022/9/16.
//


#include <iostream>
#include <ros/ros.h>
#include "range_ave_error.hpp"



int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"LIDAR_INSPECT");
    LOG(INFO) << "begin to cal error_range !!!";
    RangeAveError range_ave_error;

    ros::spin();
    return 0;
}