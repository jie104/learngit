/**
 * @file transform.hpp
 * @author zmy (626670628@qq.com)
 * @brief 一些几何变换
 * @version 0.1
 * @date 2021-04-02
 * 
 * 
 */

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "type.h"
#include <Eigen/Geometry>

namespace tf
{

    /**
     * @brief Get the Yaw object,注意返回的yaw in ranges[0:pi]
     * 
     * @tparam Quaternion 
     * @param quat 
     * @return double 
     */
    template <typename Quaternion>
    double getYaw(const Quaternion &quat)
    {
        auto euler = Eigen::Quaternionf(quat.w(), quat.x(), quat.y(), quat.z()).toRotationMatrix().eulerAngles(0, 1, 2);
        return euler[2];
    }

    template <class Quaternion>
    void EulerToQuaternion(double yaw, double roll, double pitch,
                           Quaternion &quaternion)
    {
        double cy = cos(yaw / 2.0);
        double sy = sin(yaw / 2.0);
        double cr = cos(roll / 2.0);
        double sr = sin(roll / 2.0);
        double cp = cos(pitch / 2.0);
        double sp = sin(pitch / 2.0);
        quaternion.w = cr * cp * cy + sr * sp * sy;
        quaternion.x = sr * cp * cy - cr * sp * sy;
        quaternion.y = cr * sp * cy + sr * cp * sy;
        quaternion.z = cr * cp * sy - sr * sp * cy;
    }

    template <class Quaternion>
    void EulerToEigenQuaternion(double yaw, double roll, double pitch,
                                Quaternion &quaternion)
    {
        double cy = cos(yaw / 2.0);
        double sy = sin(yaw / 2.0);
        double cr = cos(roll / 2.0);
        double sr = sin(roll / 2.0);
        double cp = cos(pitch / 2.0);
        double sp = sin(pitch / 2.0);
        quaternion.w() = cr * cp * cy + sr * sp * sy;
        quaternion.x() = sr * cp * cy - cr * sp * sy;
        quaternion.y() = cr * sp * cy + sr * cp * sy;
        quaternion.z() = cr * cp * sy - sr * sp * cy;
    }

    template <class Quaternion>
    void QuaternionToEuler(const Quaternion &quaternion, double &yaw, double &roll, double &pitch)
    {
        auto w = quaternion.w();
        auto x = quaternion.x();
        auto y = quaternion.y();
        auto z = quaternion.z();
        //    yaw = atan2(2*(w * z + x * y), 1 - 2 * (z * z + x * x));
        //    roll = asin(2*(w * x - y * z));
        //    pitch = atan2(w * y + z * x, 1 - 2 * (x * x + y * y));
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

} // namespace tf

#endif