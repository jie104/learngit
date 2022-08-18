/**
 * @file serialize_imu.h
 * @author zmy (626670628@qq.com)
 * @brief imu数据的boost非侵入式序列化方法
 * @version 0.1
 * @date 2021-04-27
 * 
 * 
 */

#ifndef SERIALIZE_IMU_H
#define SERIALIZE_IMU_H

#include "../../message/imu_msg.hpp"
using ImuData = Imu;

namespace boost
{
    namespace serialization
    {
        template <class Archive>
        void serialize(Archive &ar, ImuData &data, const unsigned int version)
        {
            ar &data.header.stamp;
            ar &data.header.sync_stamp;

            ar &data.orientation.w();
            ar &data.orientation.x();
            ar &data.orientation.y();
            ar &data.orientation.z();

            ar &data.angular_velocity.x();
            ar &data.angular_velocity.y();
            ar &data.angular_velocity.z();

            ar &data.linear_acceleration.x();
            ar &data.linear_acceleration.y();
            ar &data.linear_acceleration.z();

            ar &data.status;
        }

    } // namespace serialization
} // namespace boost

#endif
