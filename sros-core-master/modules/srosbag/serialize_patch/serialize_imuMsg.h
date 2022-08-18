//
// Created by dzl on 2022/2/26.
//

#ifndef MODULE_BAG_SERIALIZE_IMUMSG_H
#define MODULE_BAG_SERIALIZE_IMUMSG_H

#include "../../../core/msg/imu_msg.hpp"

using ImuMsg = sros::core::ImuMsg;
namespace boost {
namespace serialization {
template <class Archive>
void serialize(Archive &ar, ImuMsg &data, const unsigned int version) {
    ar &data.time_;

    ar &data.stamp_from_lidar;
    ar &data.orientation.w();
    ar &data.orientation.x();
    ar &data.orientation.y();
    ar &data.orientation.z();
    ar &data.orientation_covariance;
    ar &data.angular_velocity.x();
    ar &data.angular_velocity.y();
    ar &data.angular_velocity.z();
    ar &data.angular_velocity_covariance;
    ar &data.linear_acceleration.x();
    ar &data.linear_acceleration.y();
    ar &data.linear_acceleration.z();
    ar &data.linear_acceleration_covariance;
}
}  // namespace serialization
}  // namespace boost

#endif  // MODULE_BAG_SERIALIZE_IMUMSG_H
