/*
 * Thread.cpp
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#include "module.h"

#include "msg_bus.h"
#include "msg/str_msg.hpp"

#include <core/msg/common_msg.hpp>

namespace sros {
namespace core {

Module::Module(std::string name)
        : name_(name),
          queue_() {
    main_queue_ = MsgBus::getInstance()->getMainMsgQueue();
}

Module::~Module() {

}

void Module::start() {
    state_ = INITIALIZING;
    MsgBus::getInstance()->registerTopic("COMMAND", &queue_);
    subscribeTopic("COMMAND", CALLBACK(&Module::onCommandMsg));
    subscribeTopic("MONITOR_PING", CALLBACK(&Module::onMonitorPingMsg));
    thread_.reset(new boost::thread(boost::bind(&Module::runThread, this)));
}

void Module::runThread() {
    run();

    // 当run()返回以后，要执行stop()取消订阅的topic，否则会导致msg堆积在queue上无法释放
    LOG(INFO) << "Module run() return, ready to stop!";
    stop();
}

bool Module::subscribeTopic(topic_t topic, msg_callback_t callback) {
    LOG(INFO) << name_ << ": " << "subscribeTopic: " << topic;
    topic_map_[topic] = callback;
    MsgBus::getInstance()->registerTopic(topic, &queue_);
    return true;
}

void Module::dispatch() {
    state_ = RUNNING;
    while (state_ == RUNNING) {
        // LOG(INFO) << name_ << "'s msg queue size is " << queue_.size();
        base_msg_ptr m = queue_.get();
        if (m == nullptr) {
            LOG(ERROR) << "msg is nullptr!!!!";
        } else if (topic_map_.find(m->topic_) != topic_map_.end()) {
            topic_map_[m->topic_](m);
        }
    }
}

bool Module::pocessOnceEvent() {
    if (state_ != RUNNING) {
        return false;
    }

    base_msg_ptr m = queue_.get();
    if (m == nullptr) {
        LOG(ERROR) << "msg is nullptr!!!!";
    }

    if (topic_map_.find(m->topic_) != topic_map_.end()) {
        topic_map_[m->topic_](m);
    }
    return true;
}

void Module::sendMsg(base_msg_ptr m) {
    // LOG(INFO) << name_ << ": " << "sendMsg(): " << m->topic_;
    main_queue_->put(m);
}

void Module::waitForStartCommand() {
    LOG(INFO) << "waitForStartCommand()";
    base_msg_ptr m;
    std::string command = "";
    do {
        m = queue_.get();
        if (m == nullptr) {
            state_ = INITIALIZING;
            LOG(ERROR) << "msg is nullptr";
            break;
        }
        if (m->type_ == core::TYPE_TEST_STR) {
            command = std::dynamic_pointer_cast<core::StrMsg>(m)->data;
        }
    } while (command != "start");
    state_ = RUNNING;
}

void Module::stop() {
    state_ = STOPPED;

    unsubscribeAllTopic();

    // 清除队列中所有未处理的msg
    auto r = queue_.clear();

    LOG(INFO) << "Module " << name_ << " stopped. " << r;
}

void Module::onCommandMsg(base_msg_ptr m) {
    std::string command = std::dynamic_pointer_cast<core::StrMsg>(m)->data;

    if (command == "stop") {
        stop();
    }
}

// 响应MonitorModule的Ping消息
void Module::onMonitorPingMsg(core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);

    auto new_msg = std::make_shared<sros::core::CommonMsg>("MONITOR_PONG");
    new_msg->time_ = msg->time_;
    new_msg->str_0_ = name_;
    new_msg->int64_ = sros::core::util::get_time_in_us();

    sendMsg(new_msg);
}

bool Module::unsubscribeAllTopic() {

    // 取消订阅所有的消息
    for (auto it : topic_map_) {
        MsgBus::getInstance()->unRegisterTopic(it.first, &queue_);
    }

    topic_map_.clear();

    return true;
}

} /* namespace core */
} /* namespace SR */

