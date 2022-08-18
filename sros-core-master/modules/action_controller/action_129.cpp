//
// Created by caoyan on 1/15/21.
//

#include "action_129.h"
#include "core/logger.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action129::doStart() {
    auto cur_time = sros::core::util::get_time_in_ms();
    action_wait_timeout_ = cur_time + action_param0_ * 1000;  // 单位ms

    LOG(INFO) << "Action task will wait " << action_param0_ << "s";

}

void Action129::doTimerEvent(uint64_t cur_time) {

    if(cur_time > action_wait_timeout_) {
        doActionFinishSucceed();
    }
}
}
