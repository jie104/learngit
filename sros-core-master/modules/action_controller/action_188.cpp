//
// Created by lbx on 2022/02/24.
//

#include "action_188.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/exec_error.hpp"
#include "core/msg/command_msg.hpp"
#include "core/device/device_manager.h"
#include "rfid_manager.h"

using namespace std;
using namespace sros::core;

namespace ac {

// const int ACTION_PARAM0_GET_UP_SVC = 0x01;
// const int ACTION_PARAM0_GET_DOWN_SVC = 0x02;

void Action188::doStart() {
    if(action_param0_ == 0){
        auto results = src_sdk->getCurPgvInfo();
        if (results.empty()) {
            LOG(ERROR) << "getCurPgvInfo failed!";
            return;
        }
        LOG(INFO) << "getCurPgvInfo:"<<results.at(0)<<","<<results.at(1)<<","<<results.at(2)<<","<<results.at(3);
        if(results.at(0) == 0){
            doActionFinishFailed(ERROR_CODE_SCAN_ERROR);
        }else {
            string tag = std::to_string(results.at(0));
            LOG(INFO) << "get pgv tag:"<<tag;
            action_task_->setResultValueStr(tag);
            doActionFinishSucceed();
        }
    }else if(action_param0_ == 1){
        
        //Matrix配置增加“rfid请求epc模式“项（SS_MODE 同步读 ES_MODE 异步读）
        auto &s = sros::core::Settings::getInstance();
        std::string rfid_reqepc_mode = s.getValue<std::string>("device.rfid_reqepc_mode", "SS_MODE");

        if (rfid_reqepc_mode == "ES_MODE") {

            int rfid_reqepc_timeout = s.getValue<unsigned int>("device.rfid_reqepc_timeout", 2000);
            LOG(INFO) << "rfid_reqepc_timeout:" << rfid_reqepc_timeout;

            RfidManager::getInstance()->asyncGetRfidCmd();

            std::this_thread::sleep_for(std::chrono::milliseconds(rfid_reqepc_timeout));

            auto data = RfidManager::getInstance()->asyncGetRfidData();

            if (data.empty()) {
                LOGGER(ERROR, ACTION_TASK) << "RFID get none!";
                doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
                return;
            }

            //下发匹配串进行比较,同时兼容
            if(!action_param_str_.empty()) {
                if(data != action_param_str_) {
                    LOGGER(ERROR, ACTION_TASK) << "match fail, rfid(" << data << ") != param_str(" << action_param_str_ << ")";
                    action_task_->setResultValueStr(data);
                    doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                    return;
                }    
            }
            LOGGER(INFO, ACTION_TASK) << "get rfid(" << data << ")";
            action_task_->setResultValueStr(data);
            doActionFinishSucceed();      

        } else if(rfid_reqepc_mode == "SS_MODE") {
            // 100毫秒尝试一次尝试 10次
            for (int i = 0; i < 10; ++i) {

                auto data = RfidManager::getInstance()->syncGetRfid();
                if (data.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                } else {
                    //下发匹配串进行比较
                    if(!action_param_str_.empty()) {
                        if(data != action_param_str_) {
                            LOGGER(ERROR, ACTION_TASK) << "match fail, rfid(" << data << ") != param_str(" << action_param_str_ << ")";
                            action_task_->setResultValueStr(data);
                            doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                            return ;
                        } 
                        return;
                    }else {
                        LOGGER(INFO, ACTION_TASK) << "get rfid(" << data << ")";
                        action_task_->setResultValueStr(data);
                        doActionFinishSucceed();
                        return ;
                    }
                }
            }//

            //没有读到rfid
            LOGGER(ERROR, ACTION_TASK) << "RFID get none!";
            doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
        }

    }

}

}
