//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_130_H
#define SROS_ACTION_130_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_WAIT_IO_SIGNAL = 130,
class Action130 : public BaseAction {
 public:
    Action130() : BaseAction(ACTION_ID_WAIT_IO_SIGNAL) {}
    virtual ~Action130() {}

    void doStart() override;

    bool doCancelToInCancel() override { return true;}

    void doInCancel() override;

    void doTimerEvent (uint64_t cur_time) override;

 protected:
    Action130(int reg_act_id) : BaseAction(reg_act_id) {}

    void responseIOInput(int input_bits);

    void responseAction130(int input_bits, bool is_action_start);

};
}

#endif  // SROS_ACTION_130_H
