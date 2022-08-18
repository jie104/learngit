//
// Created by duan on 2021/6/17.
//

#ifndef SROS_SERIALIZE_LIDAR_H
#define SROS_SERIALIZE_LIDAR_H

#include "../sros-core/core/msg/laser_scan_msg.hpp"

using LidarData = sros::core::LaserScanMsg;

namespace boost{
namespace serialization{

template <class Archive>
void serialize(Archive &ar, LidarData &data, const unsigned int version) {

    ar &data.time_;

    ar &data.angle_min;
    ar &data.angle_max;
    ar &data.angle_increment;
    ar &data.time_increment;
    ar &data.scan_time;
    ar &data.range_min;
    ar &data.range_max;
    ar &data.ranges;
    ar &data.intensities;
    ar &data.sensor_name;

}

}
}


#endif  // SROS_SERIALIZE_LIDAR_H
