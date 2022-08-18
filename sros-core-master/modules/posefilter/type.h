/**
 * @file type.h
 * @author zmy (626670628@qq.com)
 * @brief 一些类型别名(方便切换框架时的类型切换)和类型相关的处理 
 * @version 0.1
 * @date 2021-03-26
 * 
 * 
 */

#ifndef POSEFILTER_MODULE_TYPE_H
#define POSEFILTER_MODULE_TYPE_H

#include "core/msg/imu_msg.hpp"
#include "core/msg/odometry_msg.hpp"
#include "core/pose.h"

namespace sros
{
    namespace pose_filter
    {
        using ImuData = sros::core::Imu;
        using ImuDataPtr = sros::core::Imu::Ptr;
        using OdomData = sros::core::nav_msgs::Odometry;
        using OdomDataPtr = sros::core::nav_msgs::Odometry::Ptr;
        // 临时使用
        using Pose2D = sros::core::Pose;

    } // namespace imu
} // namespace sros

#endif