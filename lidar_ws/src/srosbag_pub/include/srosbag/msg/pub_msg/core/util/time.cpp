/*
 * time.cpp
 *
 *  Created on: 2015年12月11日
 *      Author: lhx
 */

#include "time.h"
#include <chrono>

namespace sros {
namespace core {
namespace util {

uint64_t get_time_in_ns() {
    return static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
}

uint64_t get_time_in_us() {
    return get_time_in_ns() / 1000L;
}

uint64_t get_timestamp_in_us() {
    return static_cast<uint64_t>( static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) / 1000L);
}

uint64_t get_timestamp_in_ms() {
    return static_cast<uint64_t>( static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) / 1000000L);
}

uint64_t get_timestamp_in_s() {
    return static_cast<uint64_t>( static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) / 1000000000L);
}

uint64_t get_time_in_ms() {
    return get_time_in_ns() / 1000000L;
}

uint64_t get_time_in_s() {
    return get_time_in_ns() / 1000000000L;
}

}
}
}

