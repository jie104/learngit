/*
 * Thread.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef THREAD_H_
#define THREAD_H_

#include <string>
#include <map>
#include <vector>

#include <thread>
#include <functional>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include "msg_queue.h"

namespace sros {
namespace core {

#define CALLBACK(F) boost::bind(F, this, _1)

class Module {
public:
    Module(std::string name);
    virtual ~Module();

    void start();
    void stop();

    virtual void run() = 0;

    const std::string& getName() const;

protected:
    void runThread();

    typedef boost::function<void(base_msg_ptr)> msg_callback_t;
    bool subscribeTopic(topic_t topic, msg_callback_t callback);

    bool unsubscribeAllTopic();

    void sendMsg(base_msg_ptr m);

    void waitForStartCommand();

    void onCommandMsg(base_msg_ptr m);

    void onMonitorPingMsg(core::base_msg_ptr m);

    void dispatch();
    bool pocessOnceEvent(); // 处理一次事件循环，在没有会到dispatch情况下处理一次事件循环。类似与Qt中的QCoreApplication::processEvents()函数

    std::string name_;

    enum MODULE_STATE {
        INITIALIZING,
        RUNNING,
        STOPPED,
    };
    MODULE_STATE state_;

protected:
    typedef std::shared_ptr<boost::thread> thread_ptr;
    thread_ptr thread_;

    typedef std::map<topic_t, msg_callback_t> topic_callback_map_t;
    topic_callback_map_t topic_map_;

    MsgQueue queue_;
    msg_queue_ptr main_queue_;

};

typedef Module* module_ptr;

inline const std::string& Module::getName() const {
    return name_;
}

} /* namespace core */
} /* namespace SR */

#endif /* THREAD_H_ */
