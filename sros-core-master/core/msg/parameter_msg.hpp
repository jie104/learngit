#ifndef SROS_SET_PARAMETER_MSG_HPP
#define SROS_SET_PARAMETER_MSG_HPP

#include "base_msg.h"

namespace sros {
namespace core {

class ParameterMsg : public BaseMsg {
public:
    ParameterMsg(const topic_t & topic)
            : BaseMsg(topic, TYPE_SET_PARAMETER) {

    }

    virtual ~ParameterMsg() { };

    virtual void getTime() { };

    std::string name;
    std::string value;


};

typedef std::shared_ptr<ParameterMsg> ParameterMsg_ptr;

}
}

#endif //SROS_SET_PARAMETER_MSG_HPP
