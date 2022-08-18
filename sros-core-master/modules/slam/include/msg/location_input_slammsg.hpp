//
// Created by lfc on 17-9-11.
//

#ifndef SROS_LOCATION_INPUT_SLAMMSG_HPP
#define SROS_LOCATION_INPUT_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
#include "scan_slammsg.hpp"
#include "pose_slammsg.hpp"
namespace slam{
class LocationInputSlamMsg : public BaseSlamMsgInterface{
public:
    LocationInputSlamMsg() : BaseSlamMsgInterface(LOCATION_INPUT_SLAMMSG){

    }

    virtual ~LocationInputSlamMsg(){

    }

    ScanSlam_Ptr scan;
    PoseSlam_Ptr pose;
private:

};

typedef std::shared_ptr<LocationInputSlamMsg> LocationInput_Ptr;

}


#endif //SROS_LOCATION_INPUT_SLAMMSG_HPP
