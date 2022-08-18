/**
 * @file imu_msg.hpp
 * @author zmy (626670628@qq.com)
 * @brief IMU信息定义
 * @version 0.1
 * @date 2021-03-30
 * 
 * 
 */

#ifndef ODOD_SENSOR_MSGS_IMU_H
#define ODOD_SENSOR_MSGS_IMU_H

#include "geometry_msg.hpp"
#include "std_msg.hpp"
#include <Eigen/Geometry>
#include <memory>

struct Imu
{
    std_msgs::Header header;
    geometry_msgs::Quaternion orientation;
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Vector3 linear_acceleration;
    inline void getRPY(float &roll, float &pitch, float &yaw) const
    {
        auto w = orientation.w();
        auto x = orientation.x();
        auto y = orientation.y();
        auto z = orientation.z();

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
    using Ptr = std::shared_ptr<Imu>;
    int status{0};                                    // 0:未完成  1:正常数据  2:数据出现异常根据异常前数据推导
    static std::string msgType() { return "imuBag"; } // 用于数据录制的类型标记
    static bool isCompress() {return true;}

private:
};

inline bool operator<(const Imu::Ptr &lhs, const Imu::Ptr &rhs)
{
    return lhs->header.stamp < rhs->header.stamp;
}

inline bool operator>(const Imu::Ptr &lhs, const Imu::Ptr &rhs)
{
    return lhs->header.stamp > rhs->header.stamp;
}

inline bool operator>(const Imu &lhs, const Imu &rhs)
{
    return lhs.header.stamp > rhs.header.stamp;
}
inline bool operator<(const Imu &lhs, const Imu &rhs)
{
    return lhs.header.stamp < rhs.header.stamp;
}

#endif