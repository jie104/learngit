/**
 * @file odometry_msg.hpp
 * @author zmy (626670628@qq.com)
 * @brief Odometry信息定义
 * @version 0.1
 * @date 2021-03-30
 * 
 * 
 */

#ifndef ODOD_SENSOR_MSGS_ODOM_H
#define ODOD_SENSOR_MSGS_ODOM_H

#include "geometry_msg.hpp"
#include "std_msg.hpp"
#include <memory>
#include <string>

namespace nav_msgs
{
    struct Odometry
    {
        std_msgs::Header header;
        std::string child_frame_id;
        geometry_msgs::Pose pose;
        geometry_msgs::Twist twist;

        inline void getRPY(float &roll, float &pitch, float &yaw) const
        {
            auto w = pose.orientation.w();
            auto x = pose.orientation.x();
            auto y = pose.orientation.y();
            auto z = pose.orientation.z();

            double sinr_cosp = 2 * (w * x + y * z);
            double cosr_cosp = 1 - 2 * (x * x + y * y);
            roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (w * y - z * x);
            if (std::abs(sinp) >= 1)
                pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            yaw = std::atan2(siny_cosp, cosy_cosp);
        }
        using Ptr = std::shared_ptr<Odometry>;

        int status{0};                                         // 0:未完成  1:正常数据 2:停止状态
        static std::string msgType() { return "odometryBag"; } // 用于数据录制的类型标记
        static bool isCompress() { return true; }

    private:
    };

    inline bool operator<(const Odometry::Ptr &lhs, const Odometry::Ptr &rhs)
    {
        return lhs->header.stamp < rhs->header.stamp;
    }

    inline bool operator>(const Odometry::Ptr &lhs, const Odometry::Ptr &rhs)
    {
        return lhs->header.stamp > rhs->header.stamp;
    }

    inline bool operator<(const Odometry &lhs, const Odometry &rhs)
    {
        return lhs.header.stamp < rhs.header.stamp;
    }
    inline bool operator>(const Odometry &lhs, const Odometry &rhs)
    {
        return lhs.header.stamp > rhs.header.stamp;
    }

} // namespace sensor_msgs

#endif