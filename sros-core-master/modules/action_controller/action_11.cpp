//
// Created by caoyan on 1/15/21.
//

#include "action_11.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "rfid_manager.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action11::doStart() {
    if(action_param0_ == 0x01) {
        rfid_data_.clear();
        is_get_rfid_period_ = true;

        if (RfidManager::getInstance()->isConnected()) {
            auto getRFIDDataFun = [&]() {
              while (is_get_rfid_period_) {
                  auto data = RfidManager::getInstance()->syncGetRfid();
                  if (data.empty()) {
                      std::this_thread::sleep_for(std::chrono::milliseconds(100));
                  } else {
                      rfid_data_ = data;
                      return;
                  }
              }
            };

            std::thread thread1(getRFIDDataFun);
            thread1.detach();
        }
    } // if action_param0 == 0x01

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, action_id_, action_param0_, action_param1_);
}

bool Action11::onSrcAcFinishFirst() {
    if( action_param0_ == 0x01) {

        is_get_rfid_period_ = false;

        //下发匹配串进行比较,同时兼容
        if(!action_param_str_.empty()) {
            if(rfid_data_ != action_param_str_) {
                LOG(ERROR) << "match fail, rfid(" << rfid_data_ << ") != param_str(" << action_param_str_ << ")";
                action_task_->setResultValueStr(rfid_data_);
                doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                return false;
            }
        } else {
            action_task_->setResultValueStr(rfid_data_);
        }
    }

    return true;
}

}