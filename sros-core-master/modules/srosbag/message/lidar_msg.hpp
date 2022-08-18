//
// Created by duan on 2021/6/17.
//

#ifndef SROS_LIDAR_MSG_H
#define SROS_LIDAR_MSG_H

#include <vector>
#include <string>
#include <memory>
#include "std_msg.hpp"

namespace lidar_msg{

struct Lidar{

    std_msgs::Header header;

    double first_angle_min;
    double first_angle_max;
    double second_angle_min;
    double second_angle_max;

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
    std::string sensor_name;

    using Ptr = std::shared_ptr<Lidar>;
    int status{0};
    static std::string msgType(){return "lidarBag";}
    static bool isCompress(){return true;}
};

inline bool operator<(const Lidar::Ptr &lhs, const Lidar::Ptr &rhs){
    return lhs->header.stamp < rhs->header.stamp;
}

inline bool operator>(const Lidar::Ptr &lhs, const Lidar::Ptr &rhs){
    return lhs->header.stamp > rhs->header.stamp;
}

inline bool operator<(const Lidar &lhs, const Lidar &rhs){
    return lhs.header.stamp < rhs.header.stamp;
}

inline bool operator>(const Lidar &lhs, const Lidar &rhs){
    return lhs.header.stamp > rhs.header.stamp;
}


}

#endif  // SROS_LIDAR_MSG_H
