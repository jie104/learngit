/*
 * MyThread.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef SROS_TIMER_MODULE_H_
#define SROS_TIMER_MODULE_H_

#include "core/core.h"

#include "core/pose.h"

#include "core/msg/str_msg.hpp"

namespace sros {

class TimerModule: public core::Module {
public:
    TimerModule();
    virtual ~TimerModule();

    virtual void run();

    /* sleep hangs when roll system time back */
    void onStopTimer(core::base_msg_ptr m);
    void onStartTimer(core::base_msg_ptr m);
    void startTimer();

private:
    core::str_msg_ptr timer_msg_50ms_;
    core::str_msg_ptr timer_msg_100ms_;
    core::str_msg_ptr timer_msg_200ms_;
    core::str_msg_ptr timer_msg_1s_;
    core::str_msg_ptr timer_msg_5s_;
    core::str_msg_ptr timer_msg_20s_;

    uint64_t timer_50ms_counter_;
    uint64_t timer_100ms_counter_;
    uint64_t timer_200ms_counter_;
    uint64_t timer_1s_counter_;
    uint64_t timer_5s_counter_;
    uint64_t timer_20s_counter_;

    bool stop_timer_;
};

} /* namespace sros */

#endif /* SROS_TIMER_MODULE_H_ */
