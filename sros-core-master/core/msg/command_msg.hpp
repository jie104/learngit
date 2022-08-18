#ifndef SROS_DEBUG_COMMAND_MSG_HPP
#define SROS_DEBUG_COMMAND_MSG_HPP

#include "base_msg.h"

#include <memory>
#include <vector>
#include <list>

#include "core/pose.h"
#include "core/task/movement_task.h"
#include "core/task/action_task.h"

namespace sros {
namespace core {



class CommandMsg : public BaseMsg {
public:
    CommandMsg(const std::string & source, topic_t topic = "DEBUG_CMD")
            : BaseMsg(topic, TYPE_COMMAND), source(source) {

        param0 = 0;
        param1 = 0;
        param2 = 0;

        result_state = RESPONSE_NONE;
        result_code = ERROR_CODE_NONE;
    }

    virtual ~CommandMsg() = default;

    virtual void getTime() { };

    uint32_t req_seq = 0;
    uint64_t session_id = 0;

    CommandType command;

    std::string map_name;
    Pose pose;

    bool param_boolean = false;

    int param0 = 0;
    int param1 = 0;
    int param2 = 0;

    std::string str0;
    std::string str1;

    std::string locker_ip_address; // 独占人ip
    std::string locker_nickname; // 独占人nickname

    const std::string & source; // 该条命令的来源

    ResultState result_state = RESPONSE_NONE;
    uint32_t result_code = ERROR_CODE_NONE;

    MovementTask_ptr movement_task = nullptr;
    ActionTask_ptr action_task = nullptr;

    uint32_t mission_id = 0;
    uint64_t mission_no = 0;
    std::string user_name;
    std::string mission_cur_step_id;
    std::list<uint64_t > mission_no_lst;
    ObstacleAvoidPolicy mission_avoid_policy;

    NavigationPathi_vector paths;
    bool flag = false;
};

typedef std::shared_ptr<CommandMsg> CommandMsg_ptr;

}
}

#endif //SROS_DEBUG_COMMAND_MSG_HPP
