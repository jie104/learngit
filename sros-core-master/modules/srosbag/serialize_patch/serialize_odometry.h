/**
 * @file serialize_odometry.h
 * @author zmy (626670628@qq.com)
 * @brief odometry数据的boost非侵入式序列化方法
 * @version 0.1
 * @date 2021-04-28
 * 
 * 
 */

#ifndef SERIALIZE_ODOMETRY_H
#define SERIALIZE_ODOMETRY_H

#include "../../message/odometry_msg.hpp"
using OdomData = nav_msgs::Odometry;

namespace boost
{
    namespace serialization
    {
        template <class Archive>
        void serialize(Archive &ar, OdomData &data, const unsigned int version)
        {
            ar &data.header.stamp;
            ar &data.header.sync_stamp;

            ar &data.child_frame_id;

            ar &data.pose.position.x();
            ar &data.pose.position.y();
            ar &data.pose.position.z();

            ar &data.pose.orientation.w();
            ar &data.pose.orientation.x();
            ar &data.pose.orientation.y();
            ar &data.pose.orientation.z();

            ar &data.twist.angular.x();
            ar &data.twist.angular.y();
            ar &data.twist.angular.z();

            ar &data.twist.linear.x();
            ar &data.twist.linear.y();
            ar &data.twist.linear.z();

            ar &data.status;
        }

    } // namespace serialization
} // namespace boost

#endif