/**
 * @file scan_preprocess.h
 * @author zmy (626670628@qq.com)
 * @brief 对laser scan进行运动补偿
 * @version 0.1
 * @date 2021-06-11
 * 
 * 
 */

#ifndef LOCATION_SCAN_PREPROCESS_HPP
#define LOCATION_SCAN_PREPROCESS_HPP

#include "core/msg/laser_scan_msg.hpp"
#include "core/msg/odometry_msg.hpp"
#include "core/tf/TFOperator.h"
#include "modules/posefilter/PoseManagerModule.h"

#include <tuple>

namespace laser
{
    using OdomData = sros::core::nav_msgs::Odometry;
    class ScanPreprocess
    {
        enum class DistortionCorrectType
        {
            ODOM = 0,
            VELOCITY = 1
        };

    public:
        ScanPreprocess(slam::tf::TFOperator *tf_base_to_odo);
        ~ScanPreprocess() = default;
        void correctScan(sros::core::LaserScan_ptr scan_ptr, const bool is_upside);
        Eigen::Affine2f getDeltaLaserTF(const int64_t stamp, const int64_t delta_time) const;
        void setLaserTF(const double x, const double y, const double yaw);
        void setLaserAngle(const float angle_max, const float angle_min);
        void setDistortionCorrectType(const int type) { correct_type_ = DistortionCorrectType(type); }

    private:
       inline Eigen::Vector3f toPos(const float range, const double angle) const;
       Eigen::Vector3f extrapolatePoint(const Eigen::Vector3f &pos, const double delta_time,
                                         const Eigen::Vector3f &linear, const Eigen::Vector3f &angular,
                                         Eigen::Vector3f &new_pos, Eigen::Quaternionf &newst_quat) const;
       void toLaserScan(const std::vector<std::tuple<float, float, float>> &undistorted_points,
                         sros::core::LaserScan_ptr scan) const;

    private:
        int last_scan_cache_num_;
        float angle_max_;
        float angle_min_;
        Eigen::Quaternionf laser_quat_;
        Eigen::Vector3f laser_trans_;
        DistortionCorrectType correct_type_;
        std::vector<std::tuple<float, float, float>> last_back_ranges_;
        slam::tf::TFOperator *tf_base_to_odo_;
    };

} // namespace laser

#endif