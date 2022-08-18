/*
 * MsgQueue.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef MSGQUEUE_H_
#define MSGQUEUE_H_

#include <memory>
#include <list>
#include <mutex>
#include <condition_variable>

#include "msg/base_msg.h"

namespace sros {
namespace core {

typedef std::unique_lock<std::mutex> unique_lock;

class MsgQueue {
public:

    MsgQueue();
    virtual ~MsgQueue();

    bool put(base_msg_ptr item);
    base_msg_ptr get();
    base_msg_ptr try_get();
    size_t size() const ;

    bool clear();

private:
    mutable std::mutex mutex_;
    std::condition_variable cond_;

    std::list<base_msg_ptr> queue_;
};

typedef MsgQueue* msg_queue_ptr;
//typedef std::shared_ptr<MsgQueue> msg_queue_ptr;

extern sros::core::MsgQueue g_main_q;

} /* namespace core */
} /* namespace sros */

#endif /* MSGQUEUE_H_ */
