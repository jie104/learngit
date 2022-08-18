//
// Created by lfc on 16-1-29.
//

#ifndef POSEFILTER_POSEFILTER_H
#define POSEFILTER_POSEFILTER_H

#include "../../core/core.h"
#include "../../core/msg/PoseStampedMsg.h"
#include "../../core/pose.h"
#include "concise_odometry/concise_odometry.h"
#include "core/circle_optimizer_set.hpp"
#include "core/tf/TFOperator.h"
#include "type.h"
#include <iostream>
#include <memory>
#include "motion_state_detector.h"
#include "fusion_manager.hpp"

namespace sros
{
    namespace pose_filter
    {
        class PoseManagerModule : public core::Module
        {
        public:
            PoseManagerModule();

            virtual ~PoseManagerModule();

            virtual void run();

            void onMatchPoseMsg(sros::core::base_msg_ptr msg_ptr);

            void onOdoPoseCallback(core::Pose &p);

            void handleOdom(core::Pose p);

            sros::core::Pose addPose(sros::core::Pose &matchpose, sros::core::Pose &odosynpose, sros::core::Pose &newpose);

            static OdomData getOdomWithStamp(const uint64_t stamp);
            static std::vector<OdomData> getOdomWithStampDuration(const uint64_t stamp, const int64_t duration);

        private:
            void onParameterMsg(sros::core::base_msg_ptr m);
            void advertiseScanTF(const core::Pose &p) const;
            core::Pose updateMatchingPose(const int64_t stamp, core::Pose &last_pose, core::Pose &current_pose);
            bool dealWithOdomMsg(core::Pose &p);
            OdomData corePoseToOdometry(core::Pose &p);
            core::Pose odometryToCorePose(const OdomData &odom);
            slam::tf::TFOperator *tf_base_to_odo;
            //存储将机器人转换到odo坐标系的坐标变换
            slam::tf::TFOperator *tf_base_to_world;
            //存储将机器人转换到世界坐标系的坐标变换
            slam::tf::TFOperator *tf_scan_to_base;
            slam::tf::TFOperator *tf_fusion_to_world;

            bool get_match_pose, first_odo_pose = true;
            sros::core::PoseStampedMsg match_pose;
            sros::core::Pose odo_syn_pose, match_syn_pose, last_odo_pose, last_base_pose, match_odo_pose;
            
            int64_t stamp_match_thresh, last_odo_stamp_ = 0;
            double laser_coordx;
            //激光雷达相对于机器人中心的横坐标(朝向机器人运动方向,右手系)
            double laser_coordy; //激光雷达相对于机器人中心的纵坐标
            double laser_coordyaw;
            //激光雷达相对于机器人中心的yaw方向的偏差
            int rotate_offset;
            bool is_first_bagpose;

            circle::CircleOptimizerArray<int64_t> delta_time_stamp_;
            std::shared_ptr<sros::MotionStateDetector> state_detector_;

            double max_linear_ = 10.0; // m/s
            double max_angular_ = 3 * M_PI; // rad/s
            static std::unique_ptr<ConciseOdometry> pose_fusion_;
            const int max_stamp_cache_size_ = 200;
            core::Pose cur_pose_;
            bool deal_pose_flag_;
            std::shared_ptr<fusion::FusionManager> fusion_manager_;
        };
    }
}

#endif //POSEFILTER_POSEFILTER_H
