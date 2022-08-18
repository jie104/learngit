//
// Created by caoyan on 1/15/21.
//

#include "action_132.h"
#include "core/logger.h"
#include "rfid_manager.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action132::doStart() {
    is_in_cancel_ = false;

    if (!RfidManager::getInstance()->isConnected()) {
        LOG(ERROR) << "RFID not enable!";
        doActionFinishFailed(sros::core::ERROR_CODE_RFID_NOT_ENABLED);
    } else {
       auto getRFIDDataFun = [&](int tryTimes) {
            // 100毫秒尝试一次尝试30秒
            for (int i = 0; i < tryTimes * 10; ++i) {

                if(is_in_cancel_) {
                    return;
                }

                auto data = RfidManager::getInstance()->syncGetRfid();
                if (data.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                } else {

                    //下发匹配串进行比较,同时兼容
                    if(!action_param_str_.empty()) {
                        if(data != action_param_str_) {
                            LOG(ERROR) << "match fail, rfid(" << data << ") != param_str(" << action_param_str_ << ")";
                            action_task_->setResultValueStr(data);
                            doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                            return;
                        }
                    } 
                            
                    action_task_->setResultValueStr(data);
                    doActionFinishSucceed();
                    return;
                }
            }
            LOG(ERROR) << "RFID get none!";
            doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
        };

        std::thread thread1(getRFIDDataFun,
                            action_param0_ <= 0 ? 30 : action_param0_);  // 若没有设置尝试30秒，若设置了尝试设置的次数
        thread1.detach(); 

        
    }//else
}

void Action132::doInCancel() {
    is_in_cancel_ = true;
}

}
