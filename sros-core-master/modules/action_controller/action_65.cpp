//
// Created by caoyan on 1/15/21.
//
#include "action_65.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action65::doStart() {
    LOG(INFO) << "TEMP >>> CMD_SET_GPIO_OUTPUT: " << action_param0_;
    auto value = (uint8_t)(action_param0_ & 0x000000FF);
    src_sdk->setGPIOOuputBits(0xff, value);
    //src_sdk->setGPIOOuput(0xff00 + (uint16_t)value);

    doActionFinishSucceed();
}


}