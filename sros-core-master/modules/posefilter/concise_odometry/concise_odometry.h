/**
 * @file Concise_odometry.h
 * @author zmy (626670628@qq.com)
 * @brief 实现简单替换角度的odometry的接口,
 *        注意生成实例时使用std::shared_ptr形式
 * @version 0.1
 * @date 2021-03-30
 * 
 * 
 */

#ifndef Concise_ODOMETRY_H
#define Concise_ODOMETRY_H

#include "../type.h"
#include "odom_fusion.hpp"
#include <boost/circular_buffer.hpp>
#include <future>
#include <memory>
#include <set>
namespace sros
{
    namespace pose_filter
    {
        class ConciseOdometry
        {

        public:
            ConciseOdometry(const int data_size = 2000);
            ~ConciseOdometry();

            std::pair<Pose2D, bool> getOptionPose(int index = 0) const;
            OdomData getFusingOdometry(int index = 0) const;
            OdomData getOdometryWithStamp(uint64_t stamp) const;
            std::vector<OdomData> getOdomWithStampDuration(uint64_t stamp, int64_t duration) const;
            OdomData fuseData(const OdomData &&odom, ImuData &&imu);
            void duplicateAndUpdateTime(const OdomData &&abnormal_odom);
            inline void setAdvertiseStandstillFunc(std::function<void(const bool)> func) { advertise_standstill_ = func; }
            void run(const bool blocking = false);
            void stop();

        private:
            void fuseOdom();
            void pushOdomData(const OdomData &odom);

        private:
            size_t data_size_;
            bool stop_;
            std::function<void(const bool)> advertise_standstill_;
            std::unique_ptr<OdomFusion<OdomData, ImuData>> odom_fusion_ = nullptr;
            // boost::circular_buffer<OdomData> fusion_datas_;
            std::shared_ptr<std::multiset<OdomData, std::greater<OdomData>>> fusion_datas_ = std::make_shared<std::multiset<OdomData, std::greater<OdomData>>>();
            std::future<void> fusion_thread_;
        };
    } // namespace pose_filter

} // namespace odom

#endif