#ifndef ODOM_FUSION_HPP
#define ODOM_FUSION_HPP

#include "core/util/time.h"
#include <Eigen/Geometry>

namespace sros
{
    namespace pose_filter
    {
        template <typename OdomPose, typename Imu>
        class OdomFusion
        {

        public:
            OdomFusion();
            ~OdomFusion() = default;

            OdomPose poseFusion(const int64_t current, const OdomPose &&odom_data, const Imu &&imu_data);
            OdomPose onlyPoseFusion(const OdomPose &&odom_data, const Imu &&imu_data);
            OdomPose onlyEncoderFusion(const OdomPose &&odom_data);
            OdomPose getOdometory();
            void setOdomeory(const OdomPose &odom) { pose_ = odom; }
            void setLastOdom(const OdomPose &&odom) { last_src_pose_ = odom; }

        private:
            std::pair<Eigen::Vector3f, Eigen::Quaternionf> deltaPose(const OdomData &new_odom, const OdomData &last_odom);

        private:
            OdomPose pose_;
            OdomPose last_src_pose_;
        };

        template <typename OdomPose, typename Imu>
        OdomFusion<OdomPose, Imu>::OdomFusion()
        {
            pose_.pose = {Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()};
            pose_.header.stamp = sros::core::util::get_time_in_us();
            last_src_pose_.pose = {Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()};
            last_src_pose_.header.stamp = 0;
        }

        template <typename OdomPose, typename Imu>
        OdomPose OdomFusion<OdomPose, Imu>::poseFusion(const int64_t current, const OdomPose &&odom_data, const Imu &&imu_data)
        {
            auto delta = static_cast<double>(current - pose_.header.stamp) / 1e6; //s
            assert(delta > 0);
            auto quart_delta = 0.25 * delta;
            auto half_quat = Eigen::Quaternionf(1, imu_data.angular_velocity.x() * quart_delta,
                                                imu_data.angular_velocity.y() * quart_delta,
                                                imu_data.angular_velocity.z() * quart_delta);
            auto middle_quat = (pose_.pose.orientation * half_quat).normalized();
            pose_.pose.position += middle_quat.toRotationMatrix() * odom_data.twist.linear * delta;
            pose_.pose.orientation = imu_data.orientation; //      (middle_quat * half_quat).normalized();
            pose_.twist.linear = odom_data.twist.linear;
            pose_.twist.angular = imu_data.angular_velocity;
            pose_.header.stamp = current;
            return pose_;
        }

        template <typename OdomPose, typename Imu>
        OdomPose OdomFusion<OdomPose, Imu>::onlyPoseFusion(const OdomPose &&odom_data, const Imu &&imu_data)
        {
            auto delta = static_cast<double>(odom_data.header.stamp - pose_.header.stamp) / 1e6; //s
            assert(delta > 0);
            auto quart_delta = 0.25 * delta;
            auto half_quat = Eigen::Quaternionf(1, imu_data.angular_velocity.x() * quart_delta,
                                                imu_data.angular_velocity.y() * quart_delta,
                                                imu_data.angular_velocity.z() * quart_delta);
            auto middle_quat = (pose_.pose.orientation * half_quat).normalized();
            pose_.pose.position += middle_quat.toRotationMatrix() * deltaPose(odom_data, last_src_pose_).first;
            pose_.pose.orientation = imu_data.orientation; //      (middle_quat * half_quat).normalized();
            pose_.twist.linear = odom_data.twist.linear;
            pose_.twist.angular = imu_data.angular_velocity;
            pose_.header.stamp = odom_data.header.stamp;
            last_src_pose_ = odom_data;
            return pose_;
        }

        template <typename OdomPose, typename Imu>
        OdomPose OdomFusion<OdomPose, Imu>::getOdometory()
        {
            return pose_;
        }

        template <typename OdomPose, typename Imu>
        std::pair<Eigen::Vector3f, Eigen::Quaternionf> OdomFusion<OdomPose, Imu>::deltaPose(const OdomData &new_odom, const OdomData &last_odom)
        {
            if(last_src_pose_.header.stamp == 0)
                return std::make_pair(Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());

            auto delta = last_odom.pose.orientation.toRotationMatrix().inverse() *
                         (new_odom.pose.position - last_odom.pose.position);
            auto delta_quat = last_odom.pose.orientation.inverse() * new_odom.pose.orientation;
            // if (new_odom.header.stamp - last_odom.header.stamp > 1 / 80 || delta.norm() > 0.1 ||
            //     Eigen::Quaternionf::Identity().angularDistance(delta_quat) > 0.09)
            //     return std::make_pair(Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());

            // mid_quat = last_odom.pose.orientation.slerp(0.5, new_odom.pose.orientation);
            return std::make_pair(delta, delta_quat);
        }
        template <typename OdomPose, typename Imu>
        OdomPose OdomFusion<OdomPose, Imu>::onlyEncoderFusion(const OdomPose &&odom_data)
        {
            // auto delta = static_cast<double>(odom_data.header.stamp - pose_.header.stamp) / 1e6; //s
            // assert(delta > 0);

            auto delta_pose = deltaPose(odom_data, last_src_pose_);
            last_src_pose_ = odom_data;
            // auto half_quat = Eigen::Quaternionf::Identity().slerp(0.5, delta_pose.second);
            // auto middle_quat = (pose_.pose.orientation * half_quat).normalized();
            pose_.pose.position += pose_.pose.orientation.toRotationMatrix() * delta_pose.first;
            pose_.pose.orientation *= delta_pose.second;
            pose_.twist.linear = odom_data.twist.linear;
            pose_.twist.angular = odom_data.twist.angular;
            pose_.header.stamp = odom_data.header.stamp;
            return pose_;
        }

    } // namespace pose_filter

} // namespace sros

#endif