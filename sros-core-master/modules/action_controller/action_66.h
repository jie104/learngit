//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_66_H
#define SROS_ACTION_66_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {
//ACTION_ID_SET_GPIO_OUTPUT_DELAY = 66,
class Action66 : public BaseAction {
 public:
    Action66() : BaseAction(ACTION_ID_SET_GPIO_OUTPUT_DELAY) {}
    virtual ~Action66() {}

    void doStart() override;

    void doTimerEvent(uint64_t cur_time) override;

 private:
    uint16_t restore_value_;
    uint64_t action_wait_timeout_;
};
}

#endif  // SROS_ACTION_66_H
