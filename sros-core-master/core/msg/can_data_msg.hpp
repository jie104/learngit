/**
 * describe: 
 * Created by pengjiali on 18-12-28.
**/

#ifndef SROS_CAN_DATA_MSG_HPP
#define SROS_CAN_DATA_MSG_HPP

#include "base_msg.h"
#include <memory>

namespace sros {
namespace core {

class CanDataMsg : public BaseMsg{
public:
    CanDataMsg(topic_t topic) : BaseMsg(topic, TYPE_CAN_DATA){ };
    virtual ~CanDataMsg() {};

    virtual void getTime() {};

    int can_id = 0;

    std::vector<uint8_t> data; // 数据段

};

typedef std::shared_ptr<CanDataMsg> CanDataMsg_ptr;

}
}

#endif //SROS_CAN_DATA_MSG_HPP
