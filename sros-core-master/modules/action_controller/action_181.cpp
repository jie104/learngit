//
// Created by lbx on 2022/02/24.
//

#include "action_181.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/exec_error.hpp"
#include "core/msg/command_msg.hpp"


using namespace std;
using namespace sros::core;

namespace ac {

    /**
     * @brief action_param0_ ：使能后雷达避障 0/1
     * @brief action_param1_ ：叉臂高度
     */

void Action181::doStart() {

    auto msg2 = std::make_shared<sros::core::CommonMsg>("TOPIC_BACK_LASER_ENABLE_PUBLISH");
    msg2->flag = action_param0_;
    msg2->str_0_ = sros::device::DEVICE_UST_LIDAR_BACK;
    sros::core::MsgBus::sendMsg(msg2);

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, 24, 10, action_param1_);
    LOG(INFO) << "src_ac: no " << action_no_ << ", "
                << "id 24, p0 10, p1 " << action_param0_;
}

void Action181::doCancel() {
    src_sdk->cancelAction(action_no_);
}

void Action181::onSrcAcFinishSucceed(int result_value) {

    if (checkForkHeightFautl(action_param0_ / 1000.0)) {
        return;
    }

    doActionFinishSucceed();
}

void Action181::onSrcAcFinishFailed(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(result_value);

}

void Action181::onSrcAcFinishCanceled(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(result_value);
}


}
