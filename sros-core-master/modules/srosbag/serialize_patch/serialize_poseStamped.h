//
// Created by dzl on 2022/2/26.
//

#ifndef MODULE_BAG_SERIALIZE_POSESTAMPED_H
#define MODULE_BAG_SERIALIZE_POSESTAMPED_H

#include "../../../../sros-core/core/msg/PoseStampedMsg.h"

using PoseStampedData = sros::core::PoseStampedMsg;

namespace boost {
namespace serialization {
template <class Archive>
void serialize(Archive &ar, PoseStampedData &data, const unsigned int version) {
    ar &data.time_;

    ar &data.seq;
    ar &data.session_id;
    ar &data.pose.x();
    ar &data.pose.y();
    ar &data.pose.z();
    ar &data.pose.roll();
    ar &data.pose.pitch();
    ar &data.pose.yaw();
    ar &data.pose.confidence();
    ar &data.pose.timestamp();
    ar &data.pose.synctimestamp();
}
}  // namespace serialization
}  // namespace boost

#endif  // MODULE_BAG_SERIALIZE_POSESTAMPED_H
