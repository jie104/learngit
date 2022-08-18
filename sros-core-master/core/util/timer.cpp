//
// Created by lhx on 18-1-29.
//

#include <iostream>

#include <glog/logging.h>

#include "timer.h"

Timer::Timer(const std::string &info)
        : start_time_(0),
          end_time_(0),
          info_(info)
{
    start_time_ = sros::core::util::get_time_in_us();
}

Timer::~Timer() {
    end_time_ = sros::core::util::get_time_in_us();

    auto time_interval = end_time_ - start_time_;

//    std::cout << "[Timer] " << info_ << " " << time_interval << "us" << std::endl;
//    LOG(INFO) << "[Timer] " << info_ << " " << time_interval << "us";
}
