/*
 * MsgQueue.cpp
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#include "msg_queue.h"

#include <cassert>
#include <glog/logging.h>

namespace sros {
namespace core {

MsgQueue::MsgQueue() {

}

MsgQueue::~MsgQueue() {

}

bool MsgQueue::put(base_msg_ptr item) {
    try {
        unique_lock lock(mutex_);

        if (!item) {
            LOG(ERROR) << "Error: Item is empty!!!";
            return false;
        }

        auto to_be_removed = queue_.end();
        if (item->is_real_time_ && !queue_.empty()) {
            for (auto it = queue_.begin(); it != queue_.end(); it++) {
                if ((*it)->is_real_time_ && (*it)->topic_ == item->topic_) {
                    // 如果发现队列有相同的msg,则删除该msg,由于一直是这样尝试，所以最多只有一个相同的消息
                    to_be_removed = it;
                    break;
                }
            }
        }

        // 删除旧的msg
        if (to_be_removed != queue_.end()) {
            queue_.erase(to_be_removed);
        }

        // 把最新的msg加入队列中
        queue_.push_back(item);
        lock.unlock();

        cond_.notify_one();
    } catch (const std::system_error& ex) {
        LOG(ERROR) << ex.code() << '\n';
        LOG(ERROR) << ex.code().message() << '\n';
        LOG(ERROR) << ex.what() << '\n';
    } catch (const std::exception &e) {
        LOG(ERROR) << e.what() << '\n';
    } catch (...) {
        LOG(ERROR) << "UNNKOWE error!" << '\n';
    }
    return true;
}

base_msg_ptr MsgQueue::get() {
    unique_lock lock(mutex_);

    while (queue_.empty()) {
        cond_.wait(lock);
    }
    base_msg_ptr item = queue_.front();
    if (item != nullptr) {
        queue_.pop_front();
    }

    return item;
}

base_msg_ptr MsgQueue::try_get() {
    unique_lock lock(mutex_);

    if (queue_.empty()) {
        return nullptr;
    }

    base_msg_ptr item = queue_.front();
    queue_.pop_front();

    assert(item != nullptr);

    return item;
}

bool MsgQueue::clear() {
    unique_lock lock(mutex_);

    queue_.clear();
    return queue_.empty();
}

size_t MsgQueue::size() const {
    unique_lock lock(mutex_);

    return queue_.size();
}

} /* namespace core */
} /* namespace sros */
