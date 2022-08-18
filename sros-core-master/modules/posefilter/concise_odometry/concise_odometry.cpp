/**
 * @file concise_odometry.cpp
 * @author zmy (626670628@qq.com)
 * @brief 实现简单替换角度的odometry的实现
 * @version 0.1
 * @date 2021-03-30
 * 
 * 
 */

#include "concise_odometry.h"
#include "../transform.hpp"
#include "core/util/time.h"
#include <glog/logging.h>
#include <thread>

namespace sros
{
    namespace pose_filter
    {
        ConciseOdometry::ConciseOdometry(const int data_size)
            : data_size_(data_size),
              stop_(false),
              advertise_standstill_(nullptr)
        {
            odom_fusion_.reset(new OdomFusion<OdomData, ImuData>());
        }

        ConciseOdometry::~ConciseOdometry()
        {
        }

        std::pair<Pose2D, bool> ConciseOdometry::getOptionPose(int index) const
        {
            auto odom = getFusingOdometry(index);

            if (odom.status == 0)
                return std::make_pair(Pose2D{}, false);

            Pose2D pose;
            pose.synctimestamp() = odom.header.stamp;
            pose.timestamp() = odom.header.sync_stamp;
            pose.x() = odom.pose.position.x();
            pose.y() = odom.pose.position.y();
            pose.z() = odom.pose.position.z();
            tf::QuaternionToEuler<core::geometry_msgs::Quaternion>(odom.pose.orientation, pose.yaw(), pose.roll(), pose.pitch());

            // Pose2D pose1(odom.pose.position.x(), odom.pose.position.y(), tf::getYaw(odom.pose.orientation));

            // LOG(INFO) << pose.synctimestamp() << "---" << pose1.yaw() * 180 / M_PI << "----" << pose.yaw() * 180 / M_PI;

            return std::make_pair(pose, true);
        }

        OdomData ConciseOdometry::getFusingOdometry(int index) const
        {

            if (fusion_datas_->empty() || index > fusion_datas_->size())
                return OdomData{};

            auto it = fusion_datas_->begin();
            int i = 0;
            while (!it->status || i < index)
            {
                ++it;
                ++i;
                if (it == fusion_datas_->end())
                {
                    return OdomData{};
                }
            }
            return *it;
        }

        OdomData ConciseOdometry::getOdometryWithStamp(uint64_t stamp) const
        {

            if (fusion_datas_->empty() || stamp < fusion_datas_->crbegin()->header.stamp)
            {
                LOG(WARNING) << "without correspond odom data to stamp" << stamp;
                return OdomData{};
            }

            auto it = fusion_datas_->begin();
            if (stamp > it->header.stamp && static_cast<int64_t>(stamp) - static_cast<int64_t>(it->header.stamp) > 2.5e4)
            {
                LOG(WARNING) << "Odom DATA is too old:" << static_cast<int64_t>(stamp) - static_cast<int64_t>(it->header.stamp);
                return OdomData{};
            }

            while (!it->status || it->header.stamp > stamp)
            {
                ++it;
                if (it == fusion_datas_->end())
                {
                    return OdomData{};
                }
            }
            return *it;
        }

        std::vector<OdomData> ConciseOdometry::getOdomWithStampDuration(uint64_t stamp, int64_t duration) const
        {
            uint64_t start = std::min(stamp + duration, stamp);
            uint64_t end = std::max(stamp + duration, stamp);
            if (fusion_datas_->empty() || end < fusion_datas_->crbegin()->header.stamp)
            {
                LOG(WARNING) << "without correspond duration stamp:" << start << "--" << end << "--" << stamp;
                return std::vector<OdomData>();
            }

            auto it = fusion_datas_->begin();
            if (start > it->header.stamp && static_cast<int64_t>(start) - static_cast<int64_t>(it->header.stamp) > 2.5e4)
            {
                LOG_EVERY_N(INFO, 50) << "Odom DATA is too old to get duration:" << static_cast<int64_t>(stamp) - static_cast<int64_t>(it->header.stamp);
                return std::vector<OdomData>();
            }

            std::vector<OdomData> duration_odom;
            while (!it->status || it->header.stamp > end)
            {
                ++it;
                if (it == fusion_datas_->end())
                {
                    return std::vector<OdomData>();
                }
            }

            while (it->status && it->header.stamp >= start)
            {
                duration_odom.emplace_back(*it);
                ++it;
                if (it == fusion_datas_->end())
                {
                    LOG(WARNING) << "reached the end of odom queue!!";
                    break;
                }
            }

            std::sort(duration_odom.begin(), duration_odom.end(), [](const OdomData &lhs, const OdomData &rhs)
                      { return lhs.header.stamp < rhs.header.stamp; });

            return duration_odom;
        }

        void ConciseOdometry::fuseOdom()
        {
            // while (!stop_)
            // {
            // auto current_stamp = sros::core::util::get_time_in_us();
            // if (auto datas = sensor_handle_->getRawDatas(); datas.has_value())
            // {
            //     auto imu = datas->first;
            //     auto odom = datas->second;
            //     static Eigen::Quaternionf init_quat_inv = Eigen::Quaternionf(imu.orientation.w(), imu.orientation.x(),
            //                                                                  imu.orientation.y(), imu.orientation.z())
            //                                                   .normalized()
            //                                                   .inverse();
            //     imu.orientation = (imu.orientation * init_quat_inv).normalized();
            //     auto fused_pose = odom_fusion_->poseFusion(current_stamp, odom, imu);
            //     fusion_datas_.push_front(fused_pose);
            //     fusion_datas_.front().status = true;
            // }
            // else
            // {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
            //     continue;
            // }
            // }
        }
        void ConciseOdometry::run(bool blocking)
        {
            if (blocking)
                fuseOdom();
            else
                fusion_thread_ = std::async(std::launch::async, &ConciseOdometry::fuseOdom, this);
        }

        void ConciseOdometry::stop()
        {
            stop_ = true;
        }

        OdomData ConciseOdometry::fuseData(const OdomData &&odom, ImuData &&imu)
        {

            OdomData fused_pose;
            static uint standstill = 0;
            static bool imu_except = false;
            if (imu.status && (odom.status != 2 || standstill < 200 || std::fabs(imu.angular_velocity.z()) > 0.02))
            {
                static core::geometry_msgs::Quaternion init_quat_inv = imu.orientation.inverse().normalized();

                if (imu.status == 2)
                {
                    LOG(WARNING) << "using unbelievable imu data!!!!!";
                    imu_except = true;
                }

                if ((imu_except && imu.status == 1) ||
                    (standstill > 1000 && odom.status != 2) ||
                    (standstill > 6e4 && std::fabs(imu.angular_velocity.z()) > 0.03))
                {
                    LOG(WARNING) << "rocovery of usage imu data!";
                    // init_quat_inv = odom_fusion_->getOdometory().pose.orientation * imu.orientation.inverse();
                    init_quat_inv = getFusingOdometry().pose.orientation * imu.orientation.inverse();
                    imu_except = imu_except ? false : imu_except;
                }

                standstill = odom.status == 2 ? ++standstill : 0;
                advertise_standstill_(standstill != 0);
                
                float roll, pitch, yaw;


                auto delta_quat = init_quat_inv * imu.orientation;
                imu.getRPY(roll, pitch, yaw);
                LOG(INFO) << "原始IMU yaw: " << yaw * 180.0 / M_PI;

                imu.orientation = (delta_quat).normalized();
                // LOG(WARNING) << "has imu data";
                fused_pose = odom_fusion_->onlyPoseFusion(std::move(odom), std::move(imu));
                
                imu.getRPY(roll, pitch, yaw);
                LOG(INFO) << "被改后IMU yaw: " << yaw * 180.0 / M_PI;
                odom.getRPY(roll, pitch, yaw);
                LOG(INFO) << "融合前odo yaw: " << yaw * 180.0 / M_PI;
                fused_pose.getRPY(roll, pitch, yaw);
                LOG(INFO) << "fused_pose yaw: " << yaw * 180.0 / M_PI;
            }
            else
            {
                if (odom.status == 2)
                {
                    ++standstill;
                    advertise_standstill_(true);
                    // LOG(INFO) << "the body is at a standstill!!!" << std::endl;
                }
                else
                {
                    imu_except = true;
                    advertise_standstill_(false);
                    LOG_IF(INFO, imu.status != 1) << "without imu msgs, using src odommmmmmmmmmm!!!" << std::endl;
                }

                fused_pose = odom_fusion_->onlyEncoderFusion(std::move(odom));
            }

            float roll, pitch, yaw;
            fused_pose.getRPY(roll, pitch, yaw);
            LOG(INFO) << "fused_pose yaw: " << yaw * 180.0 / M_PI;

            fused_pose.status = 1;
            pushOdomData(fused_pose);

            return fused_pose;
        }

        void ConciseOdometry::pushOdomData(const OdomData &odom)
        {
            fusion_datas_->emplace(odom);
            if (fusion_datas_->size() > data_size_)
                fusion_datas_->erase(--fusion_datas_->end());
        }

        void ConciseOdometry::duplicateAndUpdateTime(const OdomData &&abnormal_odom)
        {
            auto pose = odom_fusion_->getOdometory();
            pose.header.stamp = abnormal_odom.header.stamp;
            pose.header.sync_stamp = abnormal_odom.header.sync_stamp;
            odom_fusion_->setOdomeory(pose);
            pushOdomData(pose);
            odom_fusion_->setLastOdom(std::move(abnormal_odom));
        }
    } // namespace pose_filter
} // namespace odom
