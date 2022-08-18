//
// Created by lfc on 17-9-1.
//

#ifndef SROS_MAP_OUTPUT_SLAMMSG_HPP
#define SROS_MAP_OUTPUT_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
#include "pose_slammsg.hpp"
namespace slam{
enum MAPPINGSLAMSTATE{
    IDLE_STATE,
    START_MAP_SLAMSTATE,
    SAVING_MAP_SLAMSTATE,
    NORMAL_MAPPING_SLAMSTATE,
    WARN_MAPPING_SLAMSTATE,
    ERR_MAPPING_SLAMSTATE,
    NOT_GET_LMK_SLAMSTATE,
    FALT_MAPPING_SLAMSTATE,
};
class MapOutputSlamMsg :public BaseSlamMsgInterface{
public:
    MapOutputSlamMsg():BaseSlamMsgInterface(MAPPING_OUTPUT_SLAMMSG){

    }

    virtual ~MapOutputSlamMsg(){

    }

    MAPPINGSLAMSTATE state;
    PoseSlam_Ptr pose;

    int64_t start_time = 0;
    int64_t stop_time = 0;
private:


};
typedef std::shared_ptr<MapOutputSlamMsg> MapOutputSlam_Ptr;

}


#endif //SROS_MAP_OUTPUT_SLAMMSG_HPP
