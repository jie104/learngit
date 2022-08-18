/**
 * @file odo_pose_msg
 *
 * @author pengjiali
 * @date 2021/4/30.
 *
 * @describe
 *
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_ODO_POSE_MSG_H
#define SROS_ODO_POSE_MSG_H

#include "base_msg.h"
#include "../pose.h"
#include <memory>

namespace sros {
namespace core {

class OdoPoseMsg: public BaseMsg {
 public:
    OdoPoseMsg(topic_t topic):BaseMsg(topic,TYPE_POSE_ODO_DATA){

    }
    virtual ~OdoPoseMsg(){}
    virtual void getTime() override { }
    Pose pose;
};
typedef std::shared_ptr<OdoPoseMsg> OdoPoseMsg_ptr;
} /* namespace core */
} /* namespace sros */

#endif  // SROS_ODO_POSE_MSG_H
