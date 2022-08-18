/*
 * MyThread.cpp
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#include <iostream>
#include <memory>

#include "core/msg/str_msg.hpp"
#include "timer_module.h"

namespace sros {

TimerModule::TimerModule() :
        Module("Timer"),
        stop_timer_(false) {
    timer_msg_50ms_.reset(new core::StrMsg("TIMER_50MS"));
    timer_msg_100ms_.reset(new core::StrMsg("TIMER_100MS"));
    timer_msg_200ms_.reset(new core::StrMsg("TIMER_200MS"));
    timer_msg_1s_.reset(new core::StrMsg("TIMER_1S"));
    timer_msg_5s_.reset(new core::StrMsg("TIMER_5S"));
    timer_msg_20s_.reset(new core::StrMsg("TIMER_20S"));

    timer_msg_50ms_->is_real_time_ = true;
    timer_msg_100ms_->is_real_time_ = true;
    timer_msg_200ms_->is_real_time_ = true;
    timer_msg_1s_->is_real_time_ = true;
    timer_msg_5s_->is_real_time_ = true;
    timer_msg_20s_->is_real_time_ = true;

    timer_50ms_counter_ = 0;
    timer_100ms_counter_ = 0;
    timer_200ms_counter_ = 0;
    timer_1s_counter_ = 0;
    timer_5s_counter_ = 0;
    timer_20s_counter_ = 0;
}

TimerModule::~TimerModule() {

}

void TimerModule::run() {
    //waitForStartCommand();
    subscribeTopic("STOP_TIMER", CALLBACK(&TimerModule::onStopTimer));
    subscribeTopic("START_TIMER", CALLBACK(&TimerModule::onStartTimer));
    LOG(INFO) << "Timer module start running";
    boost::thread(boost::bind(&TimerModule::startTimer, this));

    dispatch();
}

void TimerModule::startTimer() {
    // 下面添加了50ms延时，以保证state_已切换为RUNNING
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50)); // 50ms

    for (int i = 0; state_ == RUNNING; i++) {
        if (stop_timer_) {
            break;
        }
        boost::this_thread::sleep_for(boost::chrono::milliseconds(50)); // 50ms

        timer_50ms_counter_++;
        timer_msg_50ms_->counter = timer_50ms_counter_;
        sendMsg(timer_msg_50ms_);

        if (i % 2 == 0) {
            timer_100ms_counter_++;
            timer_msg_100ms_->counter = timer_100ms_counter_;

            sendMsg(timer_msg_100ms_);
        }

        if (i % 4 == 0) {
            timer_200ms_counter_++;
            timer_msg_200ms_->counter = timer_200ms_counter_;

            sendMsg(timer_msg_200ms_);
        }

        if (i % 20 == 0) {
            timer_1s_counter_++;
            timer_msg_1s_->counter = timer_1s_counter_;

            sendMsg(timer_msg_1s_);
        }

        if (i % 100 == 0) {
            timer_5s_counter_++;
            timer_msg_5s_->counter = timer_5s_counter_;

            sendMsg(timer_msg_5s_);
        }
        if (i % 400 == 0 ){
            timer_20s_counter_++;
            timer_msg_20s_->counter = timer_20s_counter_;
            
            sendMsg(timer_msg_20s_);
        }
    }

    LOG(INFO) <<"Timer thread exit!";
}

void TimerModule::onStopTimer(core::base_msg_ptr m) {
    stop_timer_ = true;
}

void TimerModule::onStartTimer(core::base_msg_ptr m) {
    stop_timer_ = false;
    timer_50ms_counter_ = 0;
    timer_100ms_counter_ = 0;
    timer_200ms_counter_ = 0;
    timer_1s_counter_ = 0;
    timer_5s_counter_ = 0;
    timer_20s_counter_ = 0;

    LOG(INFO) <<"Timer thread start!";
    boost::thread(boost::bind(&TimerModule::startTimer, this));
}


} /* namespace sros */
