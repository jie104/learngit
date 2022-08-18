//
// Created by lfc on 17-5-23.
//

#ifndef LMKSLAM_BASEMSGINTERFACE_HPP
#define LMKSLAM_BASEMSGINTERFACE_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <map>
namespace slam {

enum MSGTYPE {
    POSE_MSG = 1,
    SCAN_MSG = 2,
    POINT_FEATURE_MSG = 3,
    MATCH_INPUT_SET_MSG = 4,
    MATCH_OUTPUT_SET_MSG = 5,
    SENSOR_INPUT_MSG = 6,
    LOOP_OUTPUT_MSG = 7,
    SCAN_WITH_POSE_MSG = 8,
    MAP_OUTPUT_MSG = 9,
    LMK_PARA_MSG = 10,
};

class BaseMsgInterface {
public:
    BaseMsgInterface(MSGTYPE type_):msg_type(type_){

    }
    virtual ~BaseMsgInterface() {}

    MSGTYPE getType(){
        return msg_type;
    }

    int64_t stamp;
private:
    BaseMsgInterface() {}

    MSGTYPE msg_type;


};
typedef std::shared_ptr<BaseMsgInterface> BaseMsgInterface_Ptr;
typedef std::vector<BaseMsgInterface_Ptr> BaseMsgInterfaces;
//typedef std::map<BaseMsgInterface_Ptr> BaseMsgInterface_Map;



}


#endif //LMKSLAM_BASEMSGINTERFACE_HPP
