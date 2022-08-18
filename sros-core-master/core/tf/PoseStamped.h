/*
 * PoseStamped.h
 *
 *  Created on: 2016年1月24日
 *      Author: lfc
 */
#include "iostream"
#include "string"
#include "boost/shared_ptr.hpp"
#ifndef SLAM_INTERFACE_POSESTAMPED_H_
#define SLAM_INTERFACE_POSESTAMPED_H_
namespace slam {
namespace Pose {

struct Location {
	double x=0.0;
	double y=0.0;
	double z=0.0;
};
struct Rotation {
	double yaw=0.0;
	double roll=0.0;
	double pitch=0.0;
};

class PoseStamped {
public:
	PoseStamped(){}
	PoseStamped(std::string frame_,int64_t stamp_,Location position_,Rotation rotation_){}
	virtual ~PoseStamped(){}
	std::string frame;
	int64_t stamp;
	Location position;
	Rotation rotation;
};
typedef boost::shared_ptr<PoseStamped> PoseStamped_ptr;
} /* namespace Pose */
} /* namespace SROS */

#endif /* SLAM_INTERFACE_POSESTAMPED_H_ */
