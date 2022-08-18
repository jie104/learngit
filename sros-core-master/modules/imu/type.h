/**
 * @file type.h
 * @author zmy (626670628@qq.com)
 * @brief 一些类型别名(方便切换框架时的类型切换)和类型相关的处理 
 * @version 0.1
 * @date 2021-03-26
 * 
 * 
 */

#ifndef IMU_MODULE_TYPE_H
#define IMU_MODULE_TYPE_H

#include "core/msg/imu_msg.hpp"
#include "core/msg/odometry_msg.hpp"
#include <chrono>

namespace imu
{
    using ImuData = sros::core::Imu;
    using ImuDataPtr = sros::core::Imu::Ptr;
    using OdomData = sros::core::nav_msgs::Odometry;
    using OdomDataPtr = sros::core::nav_msgs::Odometry::Ptr;

} // namespace imu

#endif