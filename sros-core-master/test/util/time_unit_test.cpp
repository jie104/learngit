/**
 * @file time_unit_test
 *
 * @author pengjiali
 * @date 19-12-31.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <chrono>
#include <ctime>
#include <future>
#include <thread>
#include "core/util/time.h"

using namespace sros::core::util;

TEST(TimeUnit, ALL) {
    for (auto i = 0; i < 10; ++i) {
        auto func = [](int second) {
            std::time_t result = second;
            std::cout << result << " seconds since the Epoch" << std::asctime(std::localtime(&result));
        };

        func(get_time_in_ms() / 1000L);
        func(get_timestamp_in_ms() / 1000L);
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}