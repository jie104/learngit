//
// Created by caoyan on 1/9/21.
//

#ifndef SROS_ACTION_CONTROLLER_MODULE_H_
#define SROS_ACTION_CONTROLLER_MODULE_H_

#include <map>
#include <memory>

#include "core/core.h"
#include "src/sdk/protocol/src_protocol.h"

#include "core/pose.h"

#include "core/msg/command_msg.hpp"
#include "core/msg/str_msg.hpp"
#include "core/task/task_manager.h"

namespace ac {

class ActionControllerModule : public sros::core::Module {
 public:
    ActionControllerModule();
    virtual ~ActionControllerModule();

    virtual void run();

    void responseCommand(const sros::core::CommandMsg_ptr &msg);

 private:
    void onDebugCmdMsg(sros::core::base_msg_ptr msg);

    void onActionCmdMsg(sros::core::base_msg_ptr msg);

    void onTimer_50ms(sros::core::base_msg_ptr msg);

    void onSRCActionState(sros::core::TaskStateStruct src_action_state);

    void onNotifyMsg(sros::core::base_msg_ptr m);

    void onAlgoDetectResult(const sros::core::base_msg_ptr msg);

    void onNavBuildResult(const sros::core::base_msg_ptr msg);

    void onEacActionResult(const sros::core::base_msg_ptr msg);

    void onLaserRegionObstacleMsg(const sros::core::base_msg_ptr msg);

};

} /* namespace ac */

#endif /* SROS_ACTION_CONTROLLER_MODULE_H_ */
