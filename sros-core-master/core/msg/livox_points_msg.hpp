//
// Created by lfc on 2022/2/24.
//

#ifndef SROS_LIVOX_POINTS_MSG_HPP
#define SROS_LIVOX_POINTS_MSG_HPP

//#include <armadillo>
#include <boost/serialization/access.hpp>
#include "base_msg.h"

namespace sros {
namespace core {
class LivoxPointsMsg : public BaseMsg {
 public:
    struct CustomPoint {
        uint32_t offset_time;
        float x;
        float y;
        float z;
        uint8_t reflector;
        uint8_t tag;
        uint8_t line;

     private:
        friend class boost::serialization::access;
        template <class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar &x;
            ar &y;
            ar &z;
            ar &reflector;
            ar &tag;
            ar &line;
        }
    };

    LivoxPointsMsg() : BaseMsg("LIVOX_POINTS", TYPE_LIVOX_POINTS) {}

    virtual ~LivoxPointsMsg() {}

    void getTime() {}

    std::string header;
    uint64_t time_base;
    uint32_t point_num;
    uint8_t lidar_id;
    uint8_t rsvd[3];
    std::vector<CustomPoint> points;
};
}  // namespace core
}  // namespace sros
#endif  // SROS_LIVOX_POINTS_MSG_HPP
