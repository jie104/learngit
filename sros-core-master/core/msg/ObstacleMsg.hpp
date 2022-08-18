//
// Created by lfc on 16-1-28.
//

#ifndef SROS_OBSTACLEMSG_HPP
#define SROS_OBSTACLEMSG_HPP
#include <memory>
#include <vector>
#include "base_msg.h"
#include "../pose.h"
namespace sros {
namespace core {
class ObstacleMsg: public BaseMsg {
public:
    enum ObaState{
        STATE_OBA_STOP_0 = 0,
        STATE_OBA_STOP_1,
        STATE_OBA_STOP_2,
        STATE_OBA_STOP_3,
        STATE_OBA_STOP_4,
        STATE_OBA_SLOW,
        STATE_OBA_FREE,
    };

    ObstacleMsg(topic_t topic) : BaseMsg(topic, TYPE_OBSTACLE_DATA, false) {//禁止将is_real_time = true!!!!!
        oba_state = STATE_OBA_FREE;
        is_region_oba = false;
    };
    virtual ~ObstacleMsg(){};
    virtual void getTime(){};
    std::string oba_name;
    Location_Vector point_cloud;
    ObaState oba_state;
    bool is_region_oba = false;
    Pose car_pose;
};
typedef std::shared_ptr<ObstacleMsg> ObstacleMsg_ptr;
} /* namespace core */
} /* namespace sros */
#endif //SROS_OBSTACLEMSG_HPP
