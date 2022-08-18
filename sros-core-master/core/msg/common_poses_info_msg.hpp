//
// describe: 
// Created by pengjiali on 18-12-1.
//

#ifndef SROS_COMMON_POSES_INFO_MSG_HPP
#define SROS_COMMON_POSES_INFO_MSG_HPP

#include <memory>
#include <vector>
#include <string>
#include "base_msg.h"
#include "modules/navigation/lib/include/geometry.h"

namespace sros {
namespace core {

class CommonPosesInfoMsg: public BaseMsg {
public:
    enum class Type {
        NONE = 0x00,
        AVOID_OBSTACLE = 0x01, // 避障的信息
    };

    CommonPosesInfoMsg(topic_t topic):BaseMsg(topic,TYPE_AVOID_OBSTACLE_DATA){};
    virtual ~CommonPosesInfoMsg(){};
    virtual void getTime(){};

    Type type = Type::AVOID_OBSTACLE; // 位姿信息
    Polygons car_simulate_poses; // agv模拟的姿势
    std::string sensor_name;  //传感器名称
};
typedef std::shared_ptr<CommonPosesInfoMsg> CommonPosesInfoMsg_ptr;
} /* namespace core */
} /* namespace sros */

#endif //SROS_COMMON_POSES_INFO_MSG_HPP
