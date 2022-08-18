//
// Created by zhuqiyangz on 19-5-13.
//

#ifndef SROS_SERIALIZATION_HPP
#define SROS_SERIALIZATION_HPP

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>
#include "../stereo_point.h"

namespace boost {
namespace serialization {
template<class Archive>
void serialize(Archive &archive, StereoPoints::Point &point, const unsigned int version) {
    archive & point.x;
    archive & point.y;
    archive & point.z;
    archive & point.inten;
}
} // namespace serialization
} // namespace boost
#endif //SROS_SERIALIZATION_HPP
