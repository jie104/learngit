/*
 * TransForm.h
 *
 *  Created on: 2016年1月23日
 *      Author: lfc
 */
#include "iostream"
#include "string"
#include <eigen3/Eigen/Dense>
//#include "sros_time.h"
#include "core/pose.h"
#include "PoseStamped.h"
typedef double tfScalar;
#ifndef TRANSFORM_TRANSFORM_H_
#define TRANSFORM_TRANSFORM_H_
//TODO:要不要添加一个全局的可以设置的世界坐标系？
namespace slam {
namespace tf {
class TransForm {
 public:
    TransForm();
    TransForm(int64_t time_, sros::core::Pose pose_);
    virtual ~TransForm();

    bool inverse();
//
    bool setPoint(const sros::core::Location &point_);

    bool setRotation(const sros::core::Rotation &rot);

    bool getRotation(sros::core::Rotation& rot);
//
    bool setTransformFrom2DPose(const Eigen::Vector3d &vector_);
//
    bool getPoint(sros::core::Location& point_);
//
    bool setTimeToNow();
//
    bool setYaw(float yaw_);
//
    bool getYaw(float &yaw_);
//
    bool setTransformFromPose(const sros::core::Pose &pose_);
//
    bool setTimeStamp(const int64_t &time_);
//
    bool getTimeStamp(int64_t& time_);
//
//	bool setParentFrame(std::string parent_frame_);
////
//	bool getParentFrame(std::string &parent_frame_);
////
//	bool setChildFrame(std::string child_frame_);
////
//	bool getChildFrame(std::string &child_frame_);
//
    bool transform3DPoint(const Eigen::Vector3d &input, Eigen::Vector3d &output);
//
    bool transform2DPoint(const Eigen::Vector2d &input, Eigen::Vector2d &output);

    bool transform2DPointByInverse(const Eigen::Vector2d &input, Eigen::Vector2d &output) const;

    bool transform3DPointByInverse(const Eigen::Vector3d &input, Eigen::Vector3d &output) const;

    bool transformTF(const TransForm &input, TransForm &output) const;

    bool transformTFByInverse(const TransForm &input, TransForm &output) const;

    void quaternionToEuler(const Eigen::Quaterniond& rotate,double &yaw, double& roll, double &pitch) const;

    void buildRotateFromQuaternion(const Eigen::Quaterniond& rotate);

    void buildTFfrom2DPose(const Eigen::Vector3f& pose);
//
    bool getQuaternion(Eigen::Quaterniond& rotate_) const;

    bool get2DPose(Eigen::Vector3f& pose) const;
//
    Eigen::Quaterniond getQuaternionYXZ(tfScalar yaw, tfScalar pitch,
                                        tfScalar roll);
    Eigen::Quaterniond getQuaternionZYX(tfScalar yaw, tfScalar pitch,
                                        tfScalar roll) const ;
    int64_t pose_time;
//	std::string parent_frame;
//	std::string child_frame;
    sros::core::Location position;
    sros::core::Rotation rotation;
 private:

};

}/* namespace TF */

} /* namespace SROS */

#endif /* TRANSFORM_TRANSFORM_H_ */
