//
// Created by caoyan on 1/15/21.
//

#include "action_1.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action1::doStart() {
    if(action_param0_ == 0x01) {
        enableSVC100Camera(true, sros::device::DEVICE_SVC100_UP);  // 准备开始执行顶升动作，启用SVC100
    }

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, action_id_, action_param0_, action_param1_);
}

}