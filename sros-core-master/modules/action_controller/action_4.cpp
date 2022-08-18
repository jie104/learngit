//
// Created by caoyan on 1/15/21.
//

#include "action_4.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/msg/common_msg.hpp"

using namespace std;
using namespace sros::core;

namespace ac {

void Action4::doStart() {

    auto &s = sros::core::Settings::getInstance();
    enable_rotary_jack_keep_scan_ = (s.getValue<std::string>("inspection.enable_rotary_jack_keep_scan", "False") == "True");

    if(action_param0_ == 0x01 || action_param0_ == 0x05) {
        enableSVC100Camera(true, sros::device::DEVICE_SVC100_UP);  // 准备开始执行顶升动作，启用SVC100

        if(enable_rotary_jack_keep_scan_) {
            g_state.is_check_up_svc100_scan_state = true;
            g_state.is_forbid_send_up_svc100_to_src = false;
            LOG(INFO) << "is_check_up_svc100_scan_state: " << g_state.is_check_up_svc100_scan_state
                      << ", is_forbid_send_up_svc100_to_src: " << g_state.is_forbid_send_up_svc100_to_src;
        }

    } else if (action_param0_ == 11) {
        if (enable_rotary_jack_keep_scan_) {
            enableSVC100Camera(true, sros::device::DEVICE_SVC100_UP);  // 准备开始执行顶升动作，启用SVC100
            g_state.is_check_up_svc100_scan_state = true;
            g_state.is_forbid_send_up_svc100_to_src = true;
            LOG(INFO) << "is_check_up_svc100_scan_state: " << g_state.is_check_up_svc100_scan_state
                      << ", is_forbid_send_up_svc100_to_src: " << g_state.is_forbid_send_up_svc100_to_src;
        }
    } else if (action_param0_ == 0x02) {
        if (enable_rotary_jack_keep_scan_) {
            g_state.is_check_up_svc100_scan_state = false;
            LOG(INFO) << "is_check_up_svc100_scan_state: " << g_state.is_check_up_svc100_scan_state
                      << ", is_forbid_send_up_svc100_to_src: " << g_state.is_forbid_send_up_svc100_to_src;
        }
    }

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, action_id_, action_param0_, action_param1_);
}

bool Action4::onSrcAcFinishFirst() {
    if( action_param0_ == 0x01 || action_param0_ == 0x05) {
        auto no = g_state.cur_up_camera_offset.get().no;
        LOG(INFO) << "rack no is " << no;
        
        // 当顶升动作执行结束后，不管成功还是失败，都需要关闭SVC100
        if(!enable_rotary_jack_keep_scan_) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
        }

        auto msg = std::make_shared<CommonMsg>("TOPIC_ROTARY_JACK_UP_END");
        msg->flag = true;
        sros::core::MsgBus::sendMsg(msg);

        //下发匹配串进行比较,同时兼容
        if(!action_param_str_.empty()) {
            if(no != action_param_str_) {
                LOG(ERROR) << "match fail, rack no(" << no << ") != param_str(" << action_param_str_ << ")";
                action_task_->setResultValueStr(no);
                return false;
            }
        } else {
            action_task_->setResultValueStr(no);
        }
    } else if ( action_param0_ == 0x02) {

        auto msg = std::make_shared<CommonMsg>("TOPIC_ROTARY_JACK_DOWN_END");
        msg->flag = false;
        sros::core::MsgBus::sendMsg(msg);
    }

    return true;
}


void Action4::onSrcAcFinishSucceed(int result_value) {
    if (action_param0_ == 0x01 || action_param0_ == 0x05) {
        if(enable_rotary_jack_keep_scan_) {
            g_state.is_forbid_send_up_svc100_to_src = true;
        }
    } else if (action_param0_ == 13) {
        g_state.sync_rotate = true;
    } else if (action_param0_ == 14) {
        g_state.sync_rotate = false;
    } else if (action_param0_ == 2) {
        g_state.sync_rotate = false;

        //顶升降落后，关闭svc100
        if(enable_rotary_jack_keep_scan_) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
            g_state.is_check_up_svc100_scan_state = false;
            g_state.is_forbid_send_up_svc100_to_src = false;
        }
    }
    
    doActionFinishSucceed(result_value);
}

void Action4::onSrcAcFinishFailed(int result_value) {
    //顶升动作执行失败，关闭svc100
    if(action_param0_ == 0x01 || action_param0_ == 0x05 || action_param0_ == 0x02 || action_param0_ == 11) {
        if(enable_rotary_jack_keep_scan_) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
            g_state.is_check_up_svc100_scan_state = false;
            g_state.is_forbid_send_up_svc100_to_src = false;
            auto msg = std::make_shared<CommonMsg>("TOPIC_ROTARY_JACK_DOWN_END");
            msg->flag = false;
            sros::core::MsgBus::sendMsg(msg);

        }
    }

    doActionFinishFailed(result_value);
}

}