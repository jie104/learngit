//
// Created by caoyan on 1/15/21.
//

#include "action_134.h"
#include "core/logger.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action134::doStart() {

    LOG(INFO) << "Action task gpio overtime value " << action_param1_ << " on bit " << action_param0_;

    auto cur_time = sros::core::util::get_time_in_ms();
    action_gpio_timeout_ = cur_time + action_param1_ * 1000;  // 单位ms

    LOG(INFO) << "Action task will wait " << action_param1_ << "s";

    auto input_bits = action_param0_;

    responseAction130(input_bits, true);
}

void Action134::doInCancel() {
    auto bits = action_param0_;
    responseAction130(bits, false);
}

void Action134::doTimerEvent(uint64_t cur_time) {

    auto bits = action_param0_;
    //auto value = action_task->getActionParam1();
    auto value = 0x01; //默认高电平(1)触发，作为放行触发信号

    uint8_t input_value = (g_state.gpio_input >> bits) & (uint8_t)0x01;

    if (input_value == value) {
        //responseIOInput(bits);

        responseAction130(bits, false);

        LOG(INFO) << "ACTION_ID_OVERTIME_IO_SIGNAL succeed!";
        doActionFinishSucceed();

    } else if(cur_time > action_gpio_timeout_) {
        responseAction130(bits, false);
        LOG(ERROR) << "ACTION_ID_OVERTIME_IO_SIGNAL overtime: " << action_param1_ << "s";
        doActionFinishFailed(ERROR_CODE_WAIT_GPIO_OVERTIME);
    }

}

}
