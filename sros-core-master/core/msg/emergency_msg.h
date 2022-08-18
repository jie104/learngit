/**
 * @file emergency_msg
 *
 * @author pengjiali
 * @date 19-11-28.
 *
 * @describe  急停改变迅速发给各个模块，做到快速相应
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_EMERGENCY_MSG_H
#define SROS_EMERGENCY_MSG_H

#include "base_msg.h"
#include "core/state.h"

namespace sros {
namespace core {

class EmergencyMsg : public BaseMsg {
 public:
    EmergencyMsg() : BaseMsg("TOPIC_EMERGENCY", TYPE_EMERGENCY_STATE) {}

    virtual ~EmergencyMsg() = default;

    virtual void getTime(){};

    EmergencyState emergency_state;    // 急停状态
    EmergencySource emergency_source;  // 急停触发源
};

typedef std::shared_ptr<EmergencyMsg> EmergencyMsg_ptr;

}  // namespace core
}  // namespace sros

#endif  // SROS_EMERGENCY_MSG_H
