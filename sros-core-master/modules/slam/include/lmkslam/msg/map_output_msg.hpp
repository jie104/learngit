//
// Created by lfc on 17-8-7.
//

#ifndef PROJECT_MAP_OUTPUT_MSG_HPP
#define PROJECT_MAP_OUTPUT_MSG_HPP

#include "base_msg_interface.hpp"
#include "pose_msg.hpp"
namespace slam{
class MapOutputMsg :public BaseMsgInterface{
public:
    MapOutputMsg() : BaseMsgInterface(slam::MAP_OUTPUT_MSG) {
        successful_match = WARN_MATCH_STATE;
    }
    enum MatchState{
        NORM_MATCH_STATE = 1,
        WARN_MATCH_STATE = 3,
        ERR_MATCH_STATE = 4,
        FALT_MATCH_STATE = 5,
    };
    MatchState successful_match;
    PoseMsg_Ptr output_pose;
private:


};

typedef std::shared_ptr<MapOutputMsg> MapOutputMsg_Ptr;
}



#endif //PROJECT_MAP_OUTPUT_MSG_HPP
