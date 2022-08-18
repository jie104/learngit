//
// Created by zx on 2021/6/5.
//

#ifndef SROS_POINT_CLOUD_MSG_HPP
#define SROS_POINT_CLOUD_MSG_HPP

// INCLUDE
#include "base_msg.h"
#include <Eigen/Dense>

// CODE
namespace sros{
namespace core {

class PointCloudMsg : public BaseMsg {
 public:
    PointCloudMsg(): BaseMsg("PointCloudMsg", TYPE_LASER_SCAN_DATA) {}
    PointCloudMsg(const topic_t &topic)
        : BaseMsg(topic, TYPE_LASER_SCAN_DATA) {
        // nothing to do.
    }

    ~PointCloudMsg() = default;

    /**
     * @brief It has been abandoned.
     */
    void getTime() override { };

    /** @brief Sequence number of the massage. */
    uint32_t seq{};

    /** @brief Which sensor's data. */
    std::string sensor_name;

    /** @brief . */
    std::vector<Eigen::Vector3f> cloud;

    std::vector<uint8_t> intensities;
};

} // namespace core
} // namespace sros
#endif  // SROS_POINT_CLOUD_MSG_HPP
