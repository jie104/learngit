#include "factory/export_macro.hpp"

// EXPORT_MSG(Imu::msgType(), Imu)

// EXPORT_MSG("my_imu", Imu)

// EXPORT_MSG(nav_msgs::Odometry::msgType(), nav_msgs::Odometry)

// EXPORT_MSG(camera_msg::Camera::msgType(), camera_msg::Camera)

EXPORT_MSG("TOPIC_LASER", sros::core::LaserScanMsg)

EXPORT_MSG("FIRST_SCAN", sros::core::LaserScanMsg)

EXPORT_MSG("SECOND_SCAN", sros::core::LaserScanMsg)

EXPORT_MSG("TOPIC_D435_COLOR", sros::core::ImageMsg, false)

EXPORT_MSG("TOPIC_D435_DEPTH", sros::core::ImageMsg, false)

EXPORT_MSG("LIVOX_POINTS", sros::core::LivoxPointsMsg)

EXPORT_MSG("LIVOX_IMU", sros::core::ImuMsg)

EXPORT_MSG("OdoPoseStamped", sros::core::PoseStampedMsg)

EXPORT_MSG("TOPIC_MATCHPOSE", sros::core::PoseStampedMsg)

EXPORT_MSG("LIVOX_MID70_MSG", sros::core::LivoxPointsMsg)

EXPORT_MSG("IFM3D_IMG", sros::core::ImageMsg, false)