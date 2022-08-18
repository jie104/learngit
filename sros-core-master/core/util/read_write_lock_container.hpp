/**
 * @file ReadWriteLockContainer
 *
 * @author pengjiali
 * @date 19-12-3.
 *
 * @describe 对于一些全局变量，用此类包裹来实现快速的加读写锁功能
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_READWRITELOCKCONTAINER_H
#define SROS_READWRITELOCKCONTAINER_H

#include <boost/thread/mutex.hpp>

template <typename T>
class ReadWriteLockContainer {
 public:
    ReadWriteLockContainer() = default;
    ~ReadWriteLockContainer() = default;

    ReadWriteLockContainer(const ReadWriteLockContainer &rhs) = delete;             // copy constructor
    ReadWriteLockContainer &operator=(const ReadWriteLockContainer &rhs) = delete;  // copy assignment operator

    ReadWriteLockContainer(const ReadWriteLockContainer &&rhs) = delete;       // C++11, move constructor
    ReadWriteLockContainer &operator=(ReadWriteLockContainer &&rhs) = delete;  // C++11, move assignment operator

    T get() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return value_;
    }
    void set(const T &v) {
        boost::lock_guard<boost::shared_mutex> lock(mutex_);
        value_ = v;
    }

 private:
    T value_;
    mutable boost::shared_mutex mutex_;
};

#endif  // SROS_READWRITELOCKCONTAINER_H
