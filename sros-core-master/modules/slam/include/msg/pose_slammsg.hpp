//
// Created by lfc on 17-9-1.
//

#ifndef SROS_POSE_SLAMMSG_HPP
#define SROS_POSE_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
namespace slam{
class PoseSlamMsg :public BaseSlamMsgInterface{
public:
    PoseSlamMsg():BaseSlamMsgInterface(POSE_SLAMMSG){

    }

    virtual ~PoseSlamMsg(){

    }

    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
    float roll = 0;
    float pitch = 0;

private:


};
typedef std::shared_ptr<PoseSlamMsg> PoseSlam_Ptr;
}



#endif //SROS_POSE_SLAMMSG_HPP
