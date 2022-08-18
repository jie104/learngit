//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_129_H
#define SROS_ACTION_129_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_WAIT_TIMEOUT = 129,
class Action129 : public BaseAction {
 public:
    Action129(): BaseAction(ACTION_ID_WAIT_TIMEOUT) {}
    virtual ~Action129() {}

    void doStart() override;

    bool doCancelToInCancel() override { return true;}

    void doTimerEvent(uint64_t cur_time) override;

 private:
    uint64_t action_wait_timeout_;
};

}


#endif  // SROS_ACTION_129_H
