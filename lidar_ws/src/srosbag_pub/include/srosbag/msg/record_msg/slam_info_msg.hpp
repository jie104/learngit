//
// Created by lfc on 17-8-31.
//

#ifndef SROS_SLAM_INFO_MSG_HPP
#define SROS_SLAM_INFO_MSG_HPP

#include "base_msg.hpp"

namespace record {
class SlamInfoMsg : public BaseMsg {
public:
    SlamInfoMsg() : BaseMsg(MSG_SLAM_INFO){

    }

    virtual ~SlamInfoMsg(){

    }

    virtual bool encodeBody() override {
        encode_field(angle_min);
        encode_field(angle_max);
        encode_field(range_min);
        encode_field(range_max);
        encode_field(intensity);
        encode_field(coord_x);
        encode_field(coord_y);
        encode_field(coord_yaw);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(angle_min);
        decode_field(angle_max);
        decode_field(range_min);
        decode_field(range_max);
        decode_field(intensity);
        decode_field(coord_x);
        decode_field(coord_y);
        decode_field(coord_yaw);
        return true;
    }


    float angle_min;
    float angle_max;
    float range_min;
    float range_max;
    float intensity;
    float coord_x;
    float coord_y;
    float coord_yaw;

private:




};
typedef std::shared_ptr<SlamInfoMsg> SlamInfoMsg_ptr;

}


#endif //SROS_SLAM_INFO_MSG_HPP
