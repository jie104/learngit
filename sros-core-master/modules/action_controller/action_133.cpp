//
// Created by caoyan on 1/15/21.
//

#include "action_133.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"

using namespace std;
using namespace sros::core;

namespace ac {

const int ACTION_PARAM0_GET_UP_SVC = 0x01;
const int ACTION_PARAM0_GET_DOWN_SVC = 0x02;

void Action133::doStart() {
    auto &s = sros::core::Settings::getInstance();

    if (action_param0_ == ACTION_PARAM0_GET_UP_SVC) {
        LOG(INFO) << "Action Task get UP svc100 info";

        DMCodeOffset offset;
        g_state.cur_up_camera_offset.set(offset);

        const auto enable_up_svc100 = (s.getValue<std::string>("camera.enable_svc100_camera", "False") == "True");
        // 没有开启上视摄像头
        if (!enable_up_svc100) {
            LOG(ERROR) << "svc100 up not open!";
            doActionFinishFailed(ERROR_CODE_UP_SVC100_NOT_OPEN);
            return;
        }

        enableSVC100Camera(true, sros::device::DEVICE_SVC100_UP);  // 打开上视摄像头扫码
    } else if (action_param0_ == ACTION_PARAM0_GET_DOWN_SVC) {
        LOG(INFO) << "Action Task get DOWN svc100 info";

        DMCodeOffset offset;
        g_state.cur_down_camera_offset.set(offset);

        const auto enable_down_svc100 =
            (s.getValue<std::string>("camera.enable_svc100_down_camera", "False") == "True");
        // 没有开启下视摄像头
        if (!enable_down_svc100) {
            LOG(ERROR) << "svc100 down not open!";
            return;
        }
        enableSVC100Camera(true, sros::device::DEVICE_SVC100_DOWN);  // 打开下视摄像头扫码
    }

    auto cur_time = sros::core::util::get_time_in_ms();
    if (action_param1_ == 0) {
        action_133_wait_timeout_ = cur_time + 300;
    } else {
        action_133_wait_timeout_ = cur_time + action_param1_ * 1000;  // 单位ms
    }

}

void Action133::doTimerEvent(uint64_t cur_time) {
    if (cur_time > action_133_wait_timeout_) {
        LOG(ERROR) << "ACTION_ID_GET_SVC_INFO timeout";
        // 预定的时间到
        if (action_param0_ == ACTION_PARAM0_GET_UP_SVC) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
            doActionFinishFailed(ERROR_CODE_WAIT_UP_DMCODE_OVERTIME);
        } else if (action_param0_ == ACTION_PARAM0_GET_DOWN_SVC) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_DOWN);
            doActionFinishFailed(ERROR_CODE_WAIT_DOWN_DMCODE_OVERTIME);
        }
    } else {
        if (action_param0_ == ACTION_PARAM0_GET_UP_SVC) {
            auto device =
                sros::device::DeviceManager::getInstance()->getDeviceByName(sros::device::DEVICE_SVC100_UP);
            if (!device) {
                LOG(ERROR) << "No UP svc100 device found";
                enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
                return;
            }
            auto offset = g_state.cur_up_camera_offset.get();
            LOG(INFO) << "Get UP svc100 scan info: " << offset.no;
            if (!offset.no.empty()) {
                enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
                //下发匹配串进行比较,同时兼容
                if(!action_param_str_.empty()) {
                    if(offset.no != action_param_str_) {
                        LOG(ERROR) << "match fail, up svc100 sacn info(" << offset.no << ") != param_str(" << action_param_str_ << ")";
                        action_task_->setResultValueStr(offset.no);
                        doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                        return;
                    }
                }

                action_task_->setResultValueStr(offset.no);
                doActionFinishSucceed();
            }
        } else if (action_param0_ == ACTION_PARAM0_GET_DOWN_SVC) {
            auto device =
                sros::device::DeviceManager::getInstance()->getDeviceByName(sros::device::DEVICE_SVC100_DOWN);
            if (!device) {
                LOG(ERROR) << "No DOWN svc100 device found";
                enableSVC100Camera(false, sros::device::DEVICE_SVC100_DOWN);
            }
            auto offset = g_state.cur_down_camera_offset.get();
            LOG(INFO) << "Get DOWN svc100 scan info: " << offset.no;
            if (!offset.no.empty()) {
                enableSVC100Camera(false, sros::device::DEVICE_SVC100_DOWN);
                //下发匹配串进行比较,同时兼容
                if(!action_param_str_.empty()) {
                    if(offset.no != action_param_str_) {
                        LOG(ERROR) << "match fail, down svc100 sacn info(" << offset.no << ") != param_str(" << action_param_str_ << ")";
                        action_task_->setResultValueStr(offset.no);
                        doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                        return;
                    }
                }

                action_task_->setResultValueStr(offset.no);
                doActionFinishSucceed();
            }
        }
    }

}

}