/*
 * PoseStamped.h
 *
 *  Created on: 2016年1月25日
 *      Author: lfc
 */

#ifndef SUBPROJECTS_BAG_SROS_CORE_POSESTAMPEDMSG_H_
#define SUBPROJECTS_BAG_SROS_CORE_POSESTAMPEDMSG_H_

#include "base_msg.h"
#include "pose.h"
#include <memory>

namespace sros {
namespace bag {

class PoseStampedMsg: public BaseMsg {
public:
	PoseStampedMsg(topic_t topic):BaseMsg(topic,TYPE_POSE_STAMPED_DATA){

	}
	virtual ~PoseStampedMsg(){}
    virtual void getTime() override { }
	Pose pose;
};
typedef std::shared_ptr<PoseStampedMsg> PoseStamped_ptr;
} /* namespace core */
} /* namespace sros */

#endif /* SUBPROJECTS__SROS_CORE_POSESTAMPEDMSG_H_ */
