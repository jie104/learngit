//
// Created by dzl on 2022/2/26.
//

#ifndef SROS_SERIALIZE_LIVOX_H
#define SROS_SERIALIZE_LIVOX_H

#include "../../../core/msg/livox_points_msg.hpp"

using LivoxData = sros::core::LivoxPointsMsg;

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, LivoxData &data, const unsigned int version) {
    ar &data.time_;

    ar &data.header;
    ar &data.time_base;
    ar &data.point_num;
    ar &data.lidar_id;
    ar &data.rsvd[3];
    ar &data.points;
}
}  // namespace serialization
}  // namespace boost
#endif  // SROS_SERIALIZE_LIVOX_H
