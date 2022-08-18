/*
 * SROS.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef MSGDISPATCHER_H_
#define MSGDISPATCHER_H_


#include <string>
#include <memory>
#include <vector>
#include <map>

#include "core.h"

namespace sros {
namespace core {

class MsgBus {
public:
    static MsgBus* getInstance();
    static void sendMsg(base_msg_ptr m); // 没有继承Module的类可以通过此静态函数直接将消息推送到msgBus上，而不需要通过Module子类转发

    msg_queue_ptr getMainMsgQueue();

    bool registerTopic(topic_t topic, msg_queue_ptr queue);
    bool unRegisterTopic(topic_t topic, msg_queue_ptr queue);

    void dispatch();

    void stop();

private:
    MsgBus();
//    MsgBus(msg_queue_ptr main_msg_queue);

    typedef std::vector<msg_queue_ptr> msg_queue_ptr_vector_t;
    typedef std::map<topic_t, msg_queue_ptr_vector_t> topic_queue_map_t;
    topic_queue_map_t topic_map_;

    MsgQueue main_msg_queue_;

    enum BUS_STATE {
        RUNNING,
        STOPPED,
    };
    BUS_STATE state_;
};

typedef MsgBus* msg_bus_ptr;

} /* namespace core */
} /* namespace sros */

#endif /* MSGDISPATCHER_H_ */
