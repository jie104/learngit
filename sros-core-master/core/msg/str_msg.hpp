//
// Created by lhx on 15-12-23.
//

#ifndef SROS_TEST_STR_MSG_H
#define SROS_TEST_STR_MSG_H

#include "base_msg.h"
#include <memory>

namespace sros {
namespace core {

class StrMsg : public BaseMsg{
public:
    StrMsg(topic_t topic) : BaseMsg(topic, TYPE_TEST_STR){ };
    virtual ~StrMsg() {};

    virtual void getTime() {};

    std::string data;

    uint64_t counter;
};

typedef std::shared_ptr<StrMsg> str_msg_ptr;

}
}


#endif //SROS_TEST_STR_MSG_H
