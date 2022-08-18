//
// Created by perry on 9/29/21.
//

#include "action_137.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/msg/hmi_msg.hpp"
#include "core/exec_error.hpp"
#include "core/msg/command_msg.hpp"

using namespace std;
using namespace sros::core;

namespace ac {

const int ACTION_PARAM0_GET_UP_SVC = 0x01;
const int ACTION_PARAM0_GET_DOWN_SVC = 0x02;

void Action137::doStart() {
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
        action_137_wait_timeout_ = cur_time + 300;
    } else {
        action_137_wait_timeout_ = cur_time + action_param1_ * 1000;  // 单位ms
    }
}

void Action137::doTimerEvent(uint64_t cur_time) {
    if (cur_time > action_137_wait_timeout_) {
        LOG(ERROR) << "handle ACTION_ID_MATCH_SVC_INFO timeout";
        if (action_param0_ == ACTION_PARAM0_GET_UP_SVC) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
            doActionFinishFailed(ERROR_CODE_WAIT_UP_DMCODE_OVERTIME);
        } else if (action_param0_ == ACTION_PARAM0_GET_DOWN_SVC) {
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_DOWN);
            doActionFinishFailed(ERROR_CODE_WAIT_DOWN_DMCODE_OVERTIME);
        }
    }
    else 
    {
        if (action_param0_ == ACTION_PARAM0_GET_UP_SVC) 
        {
            doMatchSvcInfo(sros::device::DEVICE_SVC100_UP);
        } 
        else if (action_param0_ == ACTION_PARAM0_GET_DOWN_SVC) 
        {
            doMatchSvcInfo(sros::device::DEVICE_SVC100_DOWN);
        }
    }

}

void Action137::doMatchSvcInfo(const std::string& _strDev)
{
    auto device = sros::device::DeviceManager::getInstance()->getDeviceByName(_strDev);
    if (!device) {
        LOG(ERROR) << "No UP svc100 device found";
        enableSVC100Camera(false, _strDev);
        return;
    }

    DMCodeOffset offset;
    if (_strDev == sros::device::DEVICE_SVC100_UP)
    {
        offset = g_state.cur_up_camera_offset.get();
    }
    else if (_strDev == sros::device::DEVICE_SVC100_DOWN)
    {
        offset = g_state.cur_down_camera_offset.get();
    }

    bool bMatch = false;
    LOG(INFO) << "Get UP svc100 scan info: " << offset.no;
    if (!offset.no.empty() && !action_param_str_.empty()) 
    {
        //下发匹配串进行比较并兼容
        if(offset.no == action_param_str_) 
        {
            //下发串匹配了才关闭扫码
            enableSVC100Camera(false, sros::device::DEVICE_SVC100_UP);
            action_task_->setResultValueStr(offset.no);
            doActionFinishSucceed();
            bMatch = true;   
        }
    }

    //匹配失败红灯并语音报警
    if (!bMatch)
    {
        LOG(ERROR) << "match fail, up svc100 sacn info(" << offset.no << ") != param_str(" << action_param_str_ << ")";
        auto msg = make_shared<sros::core::CommandMsg>("Action137");
        msg->command = sros::core::CMD_SET_HMI_CUSTOM_STATE;
        msg->param0 = 0x01;
        msg->param1 = 1000;
        sros::core::MsgBus::sendMsg(msg);
    }
}

}
