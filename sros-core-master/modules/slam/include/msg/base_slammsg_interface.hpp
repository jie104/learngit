//
// Created by lfc on 17-5-23.
//

#ifndef NORMALSLAM_BASEMSGINTERFACE_HPP
#define NORMALSLAM_BASEMSGINTERFACE_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <map>
namespace slam {

enum SLAMMSGTYPE {
    POSE_SLAMMSG = 1,
    SCAN_SLAMMSG = 2,
    MAPPING_CMD_SLAMMSG = 3,
    MAPPING_INFO_SLAMMSG = 4,
    MAPPING_OUTPUT_SLAMMSG = 5,
    LOCATION_CMD_SLAMMSG = 6,
    LOCATION_INFO_SLAMMSG = 7,
    LOCATION_OUTPUT_SLAMMSG = 8,
    LOCATION_INPUT_SLAMMSG = 9,
    POINT_FEATURES_SLAMMSG = 10,
    LINE_SEGMENT_FEATURES_SLAMMSG = 11,
    SCAN_POINT_SLAMMSG = 12,
    MATCH_POSE_SLAMMSG = 13,
    REALTIMESLAM_CMD_SLAMMSG = 14,
    REALTIMESLAM_OUTPUT_SLAMMSG = 15,
    LOCATION_CODE_SLAMMSG = 16,
};

class BaseSlamMsgInterface {
public:
    BaseSlamMsgInterface(SLAMMSGTYPE type_):msg_type(type_){

    }
    virtual ~BaseSlamMsgInterface() {}

    SLAMMSGTYPE getType(){
        return msg_type;
    }

    int64_t stamp;
private:
    BaseSlamMsgInterface() {}

    SLAMMSGTYPE msg_type;


};
typedef std::shared_ptr<BaseSlamMsgInterface> BaseSlamMsgInterface_Ptr;
typedef std::vector<BaseSlamMsgInterface_Ptr> BaseSlamMsgInterfaces;
//typedef std::map<BaseMsgInterface_Ptr> BaseMsgInterface_Map;



}


#endif //LMKSLAM_BASEMSGINTERFACE_HPP
