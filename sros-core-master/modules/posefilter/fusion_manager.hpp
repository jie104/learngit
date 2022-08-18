//
// Created by lfc on 2021/12/8.
//

#ifndef SROS_FUSION_MANAGER_HPP
#define SROS_FUSION_MANAGER_HPP
#include "motion_state_detector.h"
#include "core/tf/transform_3d.hpp"
#include "core/msg/imu_msg.hpp"

namespace fusion{
class FusionManager {
 public:
    FusionManager(){ state_detector_ = sros::MotionStateDetector::getInstance(); }

    virtual ~FusionManager(){

    }

    const slam::tf::TransForm fusionOdoAndImu(const slam::tf::TransForm& curr_odo,const sros::core::Imu& imu){
        slam::tf::Transform3D curr_odo_3d(curr_odo),curr_pose_3d(curr_odo);
        static sros::core::Imu last_static_imu;
        if (is_first_process_) {
            initial_imu_ = imu.orientation;
            last_odo_3d_ = curr_odo_3d;
            last_pose_3d_ = curr_odo_3d;
            is_first_process_ = false;
        }
        if (imu.status == 1 &&
            (!state_detector_->staticState() || std::fabs(imu.angular_velocity.z()) > static_omega_thresh)) {
            if(is_recover_imu_process){
                auto temp_yaw_last = last_pose_3d_.yaw();
                float temp_pitch, temp_roll, temp_yaw;
                slam::tf::Transform3D temp_tf;
                sros::core::geometry_msgs::Quaternion temp_q;
                last_static_imu.getRPY(temp_roll, temp_pitch, temp_yaw);
                temp_tf.EulerToQuaternion(temp_yaw_last, temp_roll, temp_pitch, temp_q);
                last_pose_3d_.rot() = temp_q.cast<double>();

                initial_imu_ = last_pose_3d_.rot().cast<float>() * imu.orientation.inverse();
                initial_imu_.normalize();

                is_recover_imu_process = false;
            }
            sros::core::Imu delta_imu(imu);
            delta_imu.orientation = (initial_imu_ * imu.orientation);
            delta_imu.orientation.normalize();
            fusionWithImu(last_odo_3d_, last_pose_3d_, curr_odo_3d, delta_imu, curr_pose_3d);
//            LOG(INFO) << "imu tf:" << curr_pose_3d.point().x() << "," << curr_pose_3d.point().x() << ","
//                      << curr_pose_3d.yaw()<<","<<curr_odo_3d.point().x()<<","<<curr_odo_3d.point().y();
        } else {
            is_recover_imu_process = true;
            fusionWithPose(last_odo_3d_, last_pose_3d_, curr_odo_3d, curr_pose_3d);
            last_static_imu = imu;
//            LOG(INFO) << "odo tf:" << curr_pose_3d.point().x() << "," << curr_pose_3d.point().x() << ","
//                      << curr_pose_3d.yaw() <<","<< curr_odo_3d.point().x() << "," << curr_odo_3d.point().y();
        }

        last_odo_3d_ = curr_odo_3d;
        last_pose_3d_ = curr_pose_3d;
        slam::tf::TransForm return_tf;
        last_pose_3d_.toTransform(return_tf);
//        LOG_EVERY_N(INFO,100) << "tf:" << return_tf.position.x() << "," << return_tf.position.y() << ","
//                               << return_tf.rotation.yaw();
//        LOG_EVERY_N(INFO,100) << "tf:" << curr_odo.position.x() << "," << curr_odo.position.y() << ","
//                               << curr_odo.rotation.yaw();
        return return_tf;
    }

 private:
    void fusionWithImu(const slam::tf::Transform3D& last_odo_3d, const slam::tf::Transform3D& last_pose_3d,
                       const slam::tf::Transform3D& curr_odo_3d, const sros::core::Imu& imu_data,slam::tf::Transform3D& curr_pose){
//        auto delta = static_cast<double>(curr_odo_3d.stamp() - last_odo_3d.stamp()) / 1.0e6; //s
//        assert(delta > 0);
//        auto quart_delta = 0.25 * delta;
//        auto half_quat = Eigen::Quaterniond(1, imu_data.angular_velocity.x() * quart_delta,
//                                            imu_data.angular_velocity.y() * quart_delta,
//                                            imu_data.angular_velocity.z() * quart_delta);
//        auto middle_quat = (last_pose_3d.rot() * half_quat).normalized();
        slam::tf::Transform3D delta_odo;
        last_odo_3d.transformTFByInverse(curr_odo_3d, delta_odo);
        if (std::isnan(delta_odo.point()[0]) || std::isnan(delta_odo.point()[1]) || std::isnan(delta_odo.point()[2])) {
            delta_odo.point().setZero();
            LOG(INFO) << "delta odo is wrong!";
        }
        curr_pose.stamp() = curr_odo_3d.stamp();
        curr_pose.point() = last_pose_3d.point() + last_pose_3d.rot() * delta_odo.point();
        curr_pose.rot() = imu_data.orientation.cast<double>();
    }

    void fusionWithPose(const slam::tf::Transform3D& last_odo_3d, const slam::tf::Transform3D& last_pose_3d,
                        const slam::tf::Transform3D& curr_odo_3d, slam::tf::Transform3D& curr_pose) {
        slam::tf::Transform3D delta_odo;
        last_odo_3d.transformTFByInverse(curr_odo_3d, delta_odo);
        if (std::isnan(delta_odo.point()[0]) || std::isnan(delta_odo.point()[1]) || std::isnan(delta_odo.point()[2])) {
            delta_odo.point().setZero();
            LOG(INFO) << "delta odo is wrong!";
        }
        if (std::isnan(delta_odo.rot().norm())) {
            delta_odo.rot().setIdentity();
            LOG(INFO) << "delta odo rot is wrong!";
        }
        last_pose_3d.transformTF(delta_odo, curr_pose);
        curr_pose.stamp() = curr_odo_3d.stamp();
    }

    std::shared_ptr<sros::MotionStateDetector> state_detector_;
    slam::tf::Transform3D last_odo_3d_;
    slam::tf::Transform3D last_pose_3d_;
    sros::core::geometry_msgs::Quaternion initial_imu_;
    bool is_first_process_ = true;
    bool is_recover_imu_process = true;
    const double static_omega_thresh = 0.02;
};

}

#endif  // SROS_FUSION_MANAGER_HPP
