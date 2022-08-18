#ifndef SROS_COMMON_COMMAND_MSG_HPP
#define SROS_COMMON_COMMAND_MSG_HPP

#include "../pose.h"
#include "base_msg.h"
#include "core/navigation_path.h"

#include <memory>
#include <vector>

namespace sros {
namespace core {

template <typename T>
class CommonCommandMsg : public BaseMsg {
public:
    CommonCommandMsg(const topic_t & topic)
            : BaseMsg(topic, TYPE_COMMON_COMMAND) {

    }

    virtual ~CommonCommandMsg() { };

    virtual void getTime() { };

    T command;
    uint32_t seq;
    uint64_t session_id;

    std::string str0;
    std::string str1;

    Pose pose;

    int param0;
    int param1;

    bool param_boolean;

    NavigationPath_vector paths;

};

}
}

#endif //SROS_COMMON_COMMAND_MSG_HPP
