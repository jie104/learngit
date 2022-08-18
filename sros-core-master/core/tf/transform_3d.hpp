//
// Created by lfc on 2021/7/9.
//

#ifndef SROS_TRANSFORM_3D_HPP
#define SROS_TRANSFORM_3D_HPP
#include <Eigen/Dense>
#include "TransForm.h"
namespace slam {

namespace tf {
class Transform3D {
 public:
    Transform3D(const slam::tf::TransForm tf) {
        point_ = Eigen::Vector3d(tf.position.x(), tf.position.y(), tf.position.z());
        EulerToQuaternion(tf.rotation.yaw(), tf.rotation.roll(), tf.rotation.pitch(), rot_);
        stamp_ = tf.pose_time;
    }

    Transform3D(Eigen::Vector3d point = Eigen::Vector3d::Zero(), Eigen::Quaterniond r = Eigen::Quaterniond::Identity())
        : point_(point), rot_(r) {}

    bool transformPoint(const Eigen::Vector3d &input, Eigen::Vector3d &output) const {
        output = rot() * input + point();
        return true;
    }

    bool transformPointByInverse(const Eigen::Vector3d &input, Eigen::Vector3d &output) const {
        auto delta_point = input - point();
        auto rot_inverse = rot().inverse();
        output = rot_inverse * delta_point;
        return true;
    }

    bool transformTF(const Transform3D &input, Transform3D &output) const {
        output.point() = rot() * input.point() + point();
        output.rot() = rot() * input.rot();
        output.rot().normalize();
        return true;
    }

    bool transformTFByInverse(const Transform3D &input, Transform3D &output) const {
        auto delta_point = input.point() - point();
        auto rot_inverse = rot().inverse();
        output.point() = rot_inverse * delta_point;
        output.rot() = rot_inverse * input.rot();
        output.rot().normalize();
        return true;
    }

    void toTransform(slam::tf::TransForm &tf) {
        tf.position.x() = point()[0];
        tf.position.y() = point()[1];
        tf.position.z() = point()[2];
        double yaw, roll, pitch;
        QuaternionToEuler(rot(), yaw, roll, pitch);
        tf.rotation.roll() = roll;
        tf.rotation.pitch() = pitch;
        tf.rotation.yaw() = yaw;
        tf.pose_time = stamp_;
    }

    double yaw() {
        double yaw, roll, pitch;
        QuaternionToEuler(rot(), yaw, roll, pitch);
        return yaw;
    }

    double pitch() {
        double yaw, roll, pitch;
        QuaternionToEuler(rot(), yaw, roll, pitch);
        return pitch;
    }

    double roll() {
        double yaw, roll, pitch;
        QuaternionToEuler(rot(), yaw, roll, pitch);
        return roll;
    }

    static Transform3D zero() {
        Transform3D tf;
        tf.point().setZero();
        tf.rot().setIdentity();
        return tf;
    }

    template <class Quaternion>
    static void QuaternionToEuler(const Quaternion quaternion, double &yaw, double &roll, double &pitch) {
        auto w = quaternion.w();
        auto x = quaternion.x();
        auto y = quaternion.y();
        auto z = quaternion.z();
        //    yaw = atan2(2*(w * z + x * y), 1 - 2 * (z * z + x * x));
        //    roll = asin(2*(w * x - y * z));
        //    pitch = atan2(w * y + z * x, 1 - 2 * (x * x + y * y));
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    template <class Quaternion>
    static void EulerToQuaternion(double yaw, double roll, double pitch, Quaternion &quaternion) {
        double cy = cos(yaw / 2.0);
        double sy = sin(yaw / 2.0);
        double cr = cos(roll / 2.0);
        double sr = sin(roll / 2.0);
        double cp = cos(pitch / 2.0);
        double sp = sin(pitch / 2.0);
        quaternion.w() = cr * cp * cy + sr * sp * sy;
        quaternion.x() = sr * cp * cy - cr * sp * sy;
        quaternion.y() = cr * sp * cy + sr * cp * sy;
        quaternion.z() = cr * cp * sy - sr * sp * cy;
    }

    Eigen::Vector3d &point() { return point_; }

    Eigen::Quaterniond &rot() { return rot_; }

    const Eigen::Vector3d &point() const { return point_; }

    const Eigen::Quaterniond &rot() const { return rot_; }

    int64_t &stamp() { return stamp_; }

    const int64_t &stamp() const { return stamp_; }

 private:
    Eigen::Vector3d point_;
    Eigen::Quaterniond rot_;
    int64_t stamp_;
};

}  // namespace tf
}  // namespace slam
#endif  // SROS_TRANSFORM_3D_HPP
