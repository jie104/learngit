/**
 * describe: 将线程阻塞等待封装起来
 * Created by pengjiali on 18-12-13.
 * NOTE: 废弃此类，直接用std::future
**/

#ifndef SROS_ASYNC_CONDITION_VARIABLE_HPP
#define SROS_ASYNC_CONDITION_VARIABLE_HPP

#include <mutex>
#include <condition_variable>
#include <chrono>

template <class T>
class AsyncConditionVariable {
public:
    AsyncConditionVariable() = default;
    AsyncConditionVariable(const AsyncConditionVariable &) = delete;
    AsyncConditionVariable &operator=(const AsyncConditionVariable &) = delete;

    // 重置状态
    void reset() {
        std::lock_guard<std::mutex> lg(mutex_);
        ready_ = false;
    }

    T waitResult() {
        std::unique_lock<std::mutex> lk(mutex_);
        condition_variable_.wait(lk, [&] { return ready_; });
        return result_;
    }

    bool waitForResult(int duration_ms, T &result) {
        std::unique_lock<std::mutex> lk(mutex_);
        if (condition_variable_.wait_for(lk, std::chrono::milliseconds(duration_ms), [&] { return ready_; })) {
            result = std::move(result_);
            return true;
        }

        return false;
    }

    void setResult(const T &result) {
        std::lock_guard<std::mutex> lg(mutex_);
        result_ = result;
        ready_ = true;
        condition_variable_.notify_one();
    }
private:
    T result_;

    bool ready_ = false; // 记录result是否准备好
    std::mutex mutex_;
    std::condition_variable condition_variable_;
};

#endif //SROS_ASYNC_CONDITION_VARIABLE_HPP
