//
// Created by caoyan on 1/15/21.
//

#include "action_66.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action66::doStart() {
    LOG(INFO) << "TEMP >> DEBUG_SET_GPIO_OUTPUT_DELAY: " << action_param0_;
    restore_value_ = g_state.gpio_output;

    auto value = (uint8_t)(action_param0_ & 0x000000FF);
    src_sdk->setGPIOOuputBits(0xff, value);
    //src_sdk->setGPIOOuput(0xff00 + (uint16_t)value);

    auto &s = sros::core::Settings::getInstance();
    auto relay_time = s.getValue<int>("io.restore_relay_time", 1000);

    auto cur_time = sros::core::util::get_time_in_ms();
    action_wait_timeout_ = cur_time + relay_time;

}

void Action66::doTimerEvent(uint64_t cur_time) {
    if(cur_time > action_wait_timeout_) {
        auto value = (uint8_t)(restore_value_ & 0x000000FF);
        src_sdk->setGPIOOuputBits(0xff, value);
        //src_sdk->setGPIOOuput((uint16_t)0xFF00 + value);
        LOG(INFO) << "TEMP >>> CMD_SET_GPIO_OUTPUT: RESTORE to " << restore_value_;

        doActionFinishSucceed();
    }
}


}