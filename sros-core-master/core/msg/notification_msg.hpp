//
// Created by lhx on 15-12-23.
//

#ifndef SROS_NOTIFICATION_MSG_H
#define SROS_NOTIFICATION_MSG_H

#include "base_msg.h"

#include <memory>
#include <list>

#include "../task/movement_task.h"
#include "../task/action_task.h"
#include "core/mission/mission_instance.h"

namespace sros {
namespace core {

class NotificationMsg : public BaseMsg{
public:
    NotificationMsg(topic_t topic) : BaseMsg(topic, TYPE_TEST_STR){ };
    virtual ~NotificationMsg() {};

    virtual void getTime() {};

    enum NotifyType {
        NOTIFY_MOVE_TASK_FINISHED = 0,
        NOTIFY_ACTION_TASK_FINISHED = 1,
        NOTIFY_MOVE_PATH_SENT = 2,
        NOTIFY_CALIBRATION_FINISHED = 3,
        NOTIFY_UPDATE_FINISHED = 4,
        NOTIFY_MISSION_LIST_CHANGED = 5,
    };

    NotifyType notify_type;

    MovementTask_ptr movement_task;
    ActionTask_ptr action_task;
    Pose calibration;
    std::list<MissionInstancePtr> mission_list;
    uint64_t session_id;
    uint32_t seq;

    int32_t param_int;
};

typedef std::shared_ptr<NotificationMsg> NotificationMsg_ptr;

}
}


#endif //SROS_NOTIFICATION_MSG_H
