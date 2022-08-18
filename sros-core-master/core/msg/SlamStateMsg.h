/*
 * SlamStateMsg.h
 *
 *  Created on: 2016年1月25日
 *      Author: lfc
 */

#ifndef SUBPROJECTS__SROS_CORE_SLAMSTATEMSG_H_
#define SUBPROJECTS__SROS_CORE_SLAMSTATEMSG_H_

#include "core/msg/base_msg.h"
//#include "protocol_control.h"

namespace sros {
namespace core {

class SlamStateMsg : public BaseMsg {
public:
    SlamStateMsg() : BaseMsg("TOPIC_SLAMSTATE", TYPE_SLAM_STATE_DATA), progress(0) { }

    virtual ~SlamStateMsg() { }

    virtual void getTime() override { }

    SLAM_STATE_CODE slam_state;
    Progress_t progress; // 当前操作进度百分比, 0~100
};

typedef std::shared_ptr<SlamStateMsg> slam_state_msg_ptr;
} /* namespace core */
} /* namespace sros */

#endif /* SUBPROJECTS__SROS_CORE_SLAMSTATEMSG_H_ */
