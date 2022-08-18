/*
 * SROS.cpp
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#include "msg_bus.h"

namespace sros {
namespace core {

MsgBus::MsgBus()
        : state_(RUNNING),
          main_msg_queue_() {

    topic_map_.clear();
}

bool MsgBus::registerTopic(topic_t topic, msg_queue_ptr queue) {
    topic_map_[topic].push_back(queue);
    return true;
}

bool MsgBus::unRegisterTopic(topic_t topic, msg_queue_ptr queue) {
    for (auto it = topic_map_[topic].begin(); it != topic_map_[topic].end(); ++it) {
        if (*it == queue) {
            topic_map_[topic].erase(it);
            return true;
        }
    }
    return false;
}

void MsgBus::dispatch() {
    LOG(INFO) << "start dispatch";

    while (state_ == RUNNING) {
        auto m = main_msg_queue_.get(); // block here for new msg
        assert(m != nullptr);

        auto it = topic_map_.find(m->topic_);
        if (it != topic_map_.cend()) {
            for (auto msg : it->second) {
                msg->put(m);
            }
        }
    }
}

MsgBus* MsgBus::getInstance() {
    static MsgBus msg_bus;
    return &msg_bus;
}

void MsgBus::sendMsg(base_msg_ptr m) {
    getInstance()->getMainMsgQueue()->put(m);
}

msg_queue_ptr MsgBus::getMainMsgQueue() {
    return &main_msg_queue_;
}

/**
 * 设置标志后需要再收到一个消息才能停止
 */
void MsgBus::stop() {
    state_ = STOPPED;
}

} /* namespace core */
} /* namespace sros */
