//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_134_H
#define SROS_ACTION_134_H

#include "action_130.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_OVERTIME_IO_SIGNAL = 134,  // 放行超时
class Action134 : public Action130 {
 public:
    Action134() : Action130(ACTION_ID_OVERTIME_IO_SIGNAL) {}
    virtual ~Action134() {}

    void doStart() override;

    bool doCancelToInCancel() override{ return true;}

    void doInCancel() override;

    void doTimerEvent(uint64_t cur_time) override;

 private:
    uint64_t action_gpio_timeout_;  //134放行动作等待超时时间

};


}

#endif  // SROS_ACTION_134_H
