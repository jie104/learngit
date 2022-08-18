
#include "action_166.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "rfid_manager.h"
#include "core/fault_center.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action166::doStart() {

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, 24, action_param0_, action_param1_);
    LOG(INFO) << "src_ac: no " << action_no_ << ", "
                << "id 24, p0 " << action_param0_ << ", p1 " << action_param1_;

}

void Action166::doCancel() {
    src_sdk->cancelAction(action_no_);
}

void Action166::onSrcAcFinishSucceed(int result_value) {
    if (checkForkHeightFautl(action_param1_ / 1000.0)) {
        return;
    }
    
    doActionFinishSucceed();
}

void Action166::onSrcAcFinishFailed(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
}

void Action166::onSrcAcFinishCanceled(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
}

}