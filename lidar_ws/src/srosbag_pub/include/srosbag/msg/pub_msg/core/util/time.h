/*
 * time.h
 *
 *  Created on: 2015年12月11日
 *      Author: lhx
 *      NOTE: sros中获取时间都必须从此获取，严禁自己调用系统接口
 */

#ifndef SROS_CORE_UTIL_TIME_H_
#define SROS_CORE_UTIL_TIME_H_

#include <chrono>
#include <cstdint>

namespace sros {
namespace core {
namespace util {

/**
 * 以下函数，返回的系统开机时间，不会由于时间系统时间修改而改变，对时间要求连续的可以使用以下函数。
 * @return
 */

uint64_t get_time_in_ns();

uint64_t get_time_in_us();

uint64_t get_time_in_ms();

uint64_t get_time_in_s();

/**
 * 获取系统时间，该时间是从1970年开始的时间，改变时间，改时间也会随着改变，需要连续时间需求的，不允许绑定该时间
 * @return
 */
uint64_t get_timestamp_in_ns();

uint64_t get_timestamp_in_us();

uint64_t get_timestamp_in_ms();

uint64_t get_timestamp_in_s();
}  // namespace util
}  // namespace core
}  // namespace sros

#endif  // SROS_CORE_UTIL_TIME_H_
