//
// Created by caoyan on 1/15/21.
//

#include "action_70.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action70::doStart() {
    LOG(INFO) << "handleScannerAction: " << action_param0_;

    string scan_code;
    if (action_param0_ == 1) {
        auto pgv_item = action_param1_;

        if (pgv_item == 1) {
            scan_code = src_sdk->getUpPGVScanCode();
        } else if (pgv_item == 2) {
            scan_code = src_sdk->getDownPGVScanCode();
        } else {
            LOG(WARNING) << "handleScannerAction() error pgv_item : " << action_param1_;
        }

        if (scan_code.empty()) {
            doActionFinishFailed(ERROR_CODE_SCAN_ERROR);
        } else {

            //下发匹配串进行比较,同时兼容
            if(!action_param_str_.empty()) {
                if(scan_code != action_param_str_) {
                    LOG(ERROR) << "match fail, scan_code(" << scan_code << ") != param_str(" << action_param_str_ << ")";
                    action_task_->setResultValueStr(scan_code);
                    doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                } else {
                    action_task_->setResultValueStr(scan_code);
                    doActionFinishSucceed();
                }
            } else {
                action_task_->setResultValueStr(scan_code);
                doActionFinishSucceed(strtol(scan_code.c_str(), NULL, 10));
            }

        }
    } else {
        LOG(WARNING) << "handleScannerAction() error parameter: " << action_param0_;
        doActionFinishFailed(ERROR_CODE_GET_SCAN_CODE_ACTION_PARAM_INVALID);
    }

}
}