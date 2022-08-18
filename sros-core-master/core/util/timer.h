//
// Created by lhx on 18-1-29.
//

#ifndef SROS_TIMER_H
#define SROS_TIMER_H

#include <string>

#include "core/util/time.h"

/**
 * 当函数返回时，Timer自动析构，析构时会打印从Timer构造到析构的时间差
 *
 * 使用方法：
 * void function() {
 *     sros::core::util::Timer t("text");
 *
 *     function code ...
 * }
 */
class Timer {
public:
    explicit Timer(const std::string &info);
    ~Timer();

private:
    std::string info_;

    uint64_t start_time_;
    uint64_t end_time_;
};


#endif //SROS_TIMER_H
