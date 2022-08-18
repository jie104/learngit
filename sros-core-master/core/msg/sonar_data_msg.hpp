//
// Created by lhx on 16-12-01.
//

#ifndef SROS_DISTANCE_DATA_MSG_H
#define SROS_DISTANCE_DATA_MSG_H

#include "base_msg.h"
#include <memory>

namespace sros {
namespace core {

class DistanceDataMsg : public BaseMsg{
public:
    DistanceDataMsg(topic_t topic) : BaseMsg(topic, TYPE_USART_DATA){ };
    virtual ~DistanceDataMsg() {};

    virtual void getTime() {};

    std::vector<uint32_t> distances; // 单位mm
    std::string sensor_name;

};

typedef std::shared_ptr<DistanceDataMsg> DistanceDataMsg_ptr;

}
}


#endif //SROS_DISTANCE_DATA_MSG_H
