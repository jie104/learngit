/**
 * @file async_condition_variable_test
 *
 * @author pengjiali
 * @date 19-12-15.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <future>
#include <thread>
#include <chrono>
#include "core/util/async_condition_variable.hpp"

TEST(async_condition_variable_test, all_test) {
    AsyncConditionVariable<int> condition;

    auto func = [&](int intervals_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(intervals_ms));
        condition.setResult(intervals_ms);
    };

    condition.reset();
    auto t = std::thread(func, 100);
    int result = 0;
    EXPECT_TRUE(condition.waitForResult(200, result));
    EXPECT_EQ(100, result);
    t.join();

    condition.reset();
    t = std::thread(func, 300);
    result = 0;
    EXPECT_FALSE(condition.waitForResult(200, result));
    t.join();
}

/**
 * async_condition_variable_test类推荐替代品
 */
TEST(async_condition_variable, recommended_substitute) {
    auto func = [](std::promise<int> &&promise, int intervals_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(intervals_ms));
        promise.set_value(intervals_ms);
    };

    std::promise<int> promise;
    std::future<int> future = promise.get_future();
    std::thread(func, std::move(promise), 100).detach();
    std::future_status status = future.wait_for(std::chrono::milliseconds(200));
    EXPECT_EQ(status, std::future_status::ready);
    EXPECT_EQ(future.get(), 100);

    promise = std::promise<int>();
    future = promise.get_future();
    std::thread(func, std::move(promise), 300).detach();
    status = future.wait_for(std::chrono::milliseconds(200));
    EXPECT_EQ(status, std::future_status::timeout);
}