
#ifndef SDQUEUE_H
#define QUESDQUEUE_HUE_H

#include <mutex>
#include <list>
#include <condition_variable>

#define Q_MAX_SIZE 1024

namespace sros {
namespace core {

typedef std::unique_lock<std::mutex> unique_lock;

template<typename T>
class SDQueue {
public:
SDQueue(uint32_t size = Q_MAX_SIZE) : max_size_(size) {
}

bool push(const T &item, int timeout = 3) {
    unique_lock locker(mutex_);
    while (list_.size() >= max_size_) {
        if (cond_.wait_for(locker, std::chrono::seconds(timeout)) == std::cv_status::timeout) {
            return false;
        }
    } 

    list_.push_back(item);
    cond_.notify_one();
    return true;
}

bool try_push(const T &item) {
    unique_lock locker(mutex_);
    if (list_.size() >= max_size_) {
        return false;
    }

    list_.push_back(item);
    cond_.notify_one();
    return true;
}

T pop() {
    unique_lock locker(mutex_);
    while (list_.empty()) {
        cond_.wait(locker);
    }
    T item = list_.front();
    list_.pop_front();
    cond_.notify_one();
    return item;
}

T try_pop() {
    unique_lock locker(mutex_);
    if (list_.empty())
    {
        return nullptr;
    }
    T item = list_.front();
    list_.pop_front();
    cond_.notify_one();
    return item;
}

void wait_until_not_empty() {
    unique_lock locker(mutex_);
    while (list_.empty()) {
        cond_.wait(locker);
    }
}

void all_items(std::list<T> &lst) {
    unique_lock locker(mutex_);
    for (auto item : list_) {
        lst.push_back(item);
    }
    return;
}

template <typename _StrictWeakOrdering>
void sort(_StrictWeakOrdering _cmp) {
    unique_lock locker(mutex_);
    list_.sort(_cmp);
}

uint32_t size() {
    unique_lock locker(mutex_);
    return list_.size();
}

T remove_item(uint64_t id) {
    unique_lock locker(mutex_);
    auto to_be_removed = list_.end();
    if (list_.empty()) {
        return nullptr;
    }

    for (auto it = list_.begin(); it != list_.end(); it++) {
        if ((*it)->getNo() == id) {
            to_be_removed = it;
            break;
        }
    }

    if (to_be_removed != list_.end()) {
        auto ret = (*to_be_removed);
        list_.erase(to_be_removed);
        return ret;
    }
    return nullptr;
}

bool has_item(uint64_t id) {
    unique_lock locker(mutex_);
    for (auto item : list_) {
        if (item->getNo() == id) {
            return true;
        }
    }
    return false;
}

bool clear() {
    unique_lock locker(mutex_);
    // TODO 如果是指针，内存泄露?,使用智能指针，可以解决该问题
    list_.clear();
    return true;
}

private:

private:
    uint32_t max_size_;
    std::list<T> list_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

}
}

#endif
