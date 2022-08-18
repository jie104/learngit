#ifndef SROS_COMMON_MSG_HPP
#define SROS_COMMON_MSG_HPP

#include "base_msg.h"
#include "../pose.h"
#include "core/navigation_path.h"

#include <memory>
#include <vector>

namespace sros {
namespace core {

class CommonMsg : public BaseMsg {
public:
    CommonMsg(topic_t topic) : BaseMsg(topic, TYPE_COMMON) { }

    virtual ~CommonMsg() { };

    virtual void getTime() { };

    std::string str_0_;
    std::string str_1_;

    Pose pose_;

    int int_0_;
    int int_1_;

    int64_t int64_;

    NavigationPath_vector paths_;

    bool flag = false;
};

}
}

#endif //SROS_COMMON_MSG_HPP
