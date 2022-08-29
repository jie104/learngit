//
// Created by lfc on 17-8-31.
//

#ifndef SROS_BAG_SLAM_INFO_MSG_HPP
#define SROS_BAG_SLAM_INFO_MSG_HPP

#include "base_msg.h"
namespace sros{
namespace bag{
class SlamInfoMsg : public BaseMsg {
public:
    SlamInfoMsg() : BaseMsg("SLAM_INFO",TYPE_SLAM_INFO_DATA){

    }

    virtual ~SlamInfoMsg(){

    }

    virtual void getTime(){

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

}


#endif //SROS_SLAM_INFO_MSG_HPP
