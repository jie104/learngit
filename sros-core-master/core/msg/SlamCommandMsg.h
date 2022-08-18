/*
 * SlamCommandMsg.h
 *
 *  Created on: 2016年1月25日
 *      Author: lfc
 */

#ifndef SUBPROJECTS__SROS_CORE_SLAMCOMMANDMSG_H_
#define SUBPROJECTS__SROS_CORE_SLAMCOMMANDMSG_H_

#include "base_msg.h"
#include "core/pose.h"
namespace sros {
namespace core {

class SlamCommandMsg : public BaseMsg {
public:
    SlamCommandMsg() : BaseMsg("TOPIC_SLAMCOMMAND", TYPED_SLAM_COMMAND_DATA) { }

    virtual ~SlamCommandMsg() { }

    virtual void getTime() override { }

    SLAM_COMMAND_TYPE slam_command;
    std::string map_name;
    std::string map_path;
    sros::core::Pose pose;
    bool use_curr_pose = false;
};

typedef std::shared_ptr<SlamCommandMsg> slam_command_msg_ptr;

} /* namespace core */
} /* namespace sros */

#endif /* SUBPROJECTS__SROS_CORE_SLAMCOMMANDMSG_H_ */

