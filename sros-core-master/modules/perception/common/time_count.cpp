//
// Created by zx on 2020/10/26.
//

#include "time_count.h"

std::unordered_map<std::string, TimeCount::TimePoint> TimeCount::g_tick_time_points = {};
std::unordered_map<std::string, TimeCount::TimeDetail> TimeCount::g_details = {};
