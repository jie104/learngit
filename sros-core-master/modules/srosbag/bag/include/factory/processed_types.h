/**
 * @file msg_config.h
 * @author zmy (626670628@qq.com)
 * @brief 用于设置哪些message可以record 和 play
 * @version 0.1
 * @date 2021-05-17
 *
 *
 */

#ifndef PROCESSED_TYPES_H
#define PROCESSED_TYPES_H

#include "type_container.hpp"

// #include "../../message/imu_msg.hpp"
// #include "../../message/odometry_msg.hpp"
//#include "../../message/camera_msg.hpp"
//#include "../../message/lidar_msg.hpp"
#include "../sros-core/core/msg/laser_scan_msg.hpp"
#include "../sros-core/core/msg/PoseStampedMsg.h"
#include "../sros-core/core/msg/image_msg.hpp"
#include "../sros-core/core/msg/imu_msg.hpp"
#include "../sros-core/core/msg/laser_scan_msg.hpp"
#include "../sros-core/core/msg/livox_points_msg.hpp"

HANDLE_TYPES(
    // Imu,
    // nav_msgs::Odometry,
    sros::core::LaserScanMsg,
    sros::core::ImageMsg,
    sros::core::PoseStampedMsg,
    sros::core::LivoxPointsMsg,
    sros::core::ImuMsg
    )

#endif
