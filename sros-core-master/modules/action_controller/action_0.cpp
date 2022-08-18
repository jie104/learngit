//
// Created by caoyan on 1/15/21.
//

#include "action_0.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action0::doStart() {

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, action_id_, action_param0_, action_param1_);
}

}