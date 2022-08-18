//
// Created by caoyan on 4/9/21.
//

#include "action_163.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "rfid_manager.h"
#include "core/fault_center.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action163::doStart() {

    is_forklift_running_ = false;

    auto& s = sros::core::Settings::getInstance();
    enable_action_check_pallet_signal_ = (s.getValue<string>("forklift.enable_action_check_pallet_signal", "True") == "True");

    // 检测到位信号是否触发
    if(enable_action_check_pallet_signal_ && (!checkPalletInPlaceSignal())) {
        LOG(ERROR) << "checkPalletInPlaceSignal: false";
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        return;
    }

    if(isForkControlTypeSrc()) {
        // 向SRC发送动作指令
        src_sdk->executeAction(action_no_, 24, 4, action_param0_);
        LOG(INFO) << "src_ac: no " << action_no_ << ", "
                  << "id 24, p0 4, p1 " << action_param0_;
    } else {

        if(!isEnableEac()) {
            doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
            return;
        }

        sendEacActionTask(action_no_, 207, 1, action_param0_);
        LOG(INFO) << "eac_ac: no " << action_no_ << ", "
                  << "id 207, p0 1, p1 " << action_param0_;
    }

    
    is_forklift_running_ = true;

}

void Action163::doCancel() {
    if(is_forklift_running_) {
        if(isForkControlTypeSrc()) {
            src_sdk->cancelAction(action_no_);
        } else {
            cancelEacActionTask(action_no_);
        }

        is_forklift_running_ = false;
    }

}

void Action163::onSrcAcFinishSucceed(int result_value) {

    if (checkForkHeightFautl(action_param0_ / 1000.0)) {
        return;
    }

    is_forklift_running_ = false;
    // 检测到位信号是否触发
    if(enable_action_check_pallet_signal_ && (!checkPalletInPlaceSignal())) {
        LOG(ERROR) << "checkPalletInPlaceSignal: false";
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        return;
    }
    
    doActionFinishSucceed();

    //开启载货移动过程中，检测栈板信号
    auto& s = sros::core::Settings::getInstance();
    bool enable_loading_check_pallet_signal = (s.getValue<string>("forklift.enable_loading_check_pallet_signal", "False") == "True");
    if (enable_loading_check_pallet_signal) {
        g_state.is_loading_check_pallet_signal_ = true;
    } else {
        g_state.is_loading_check_pallet_signal_ = false;
    }

}

void Action163::onSrcAcFinishFailed(int result_value) {
    LOG(ERROR) << "action failed, result_value:"<<result_value;
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKUP_GOODS);

}

void Action163::onSrcAcFinishCanceled(int result_value) {
    LOG(ERROR) << "action cancled, result_value:"<<result_value;
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKUP_GOODS);
}

}