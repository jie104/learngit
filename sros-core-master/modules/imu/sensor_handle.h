/**
 * @file sensor_handle.cpp
 * @author zmy (626670628@qq.com)
 * @brief 用于odom，imu融合前的数据获取及处理
 * @version 0.1
 * @date 2021-03-31
 * 
 * 
 */

#include "core/circle_optimizer_set.hpp"
#include "driver_base.h"
#include "type.h"
#include "core/circle_optimizer_set.hpp"
#include "core/tf/imu_circle_array.hpp"
#include <Eigen/Geometry>
#include <deque>
#include <memory>
#include <set>
#include <vector>
// #include <optional>

#ifndef SENSOR_HANDLE_H
#define SENSOR_HANDLE_H

#include "core/device/device.h"
#include "../posefilter/motion_state_detector.h"

namespace Eigen
{
    using Vector6f = Matrix<float, 6, 1>;
} // namespace Eigen

namespace imu
{
    class SensorHandle : public std::enable_shared_from_this<SensorHandle>
    {
        enum class ImuType
        {
            NONE = 0,
            BAG = 1,
            ATOM = 2,
            LINS = 3,
            ASENSING = 4,
            ASENSING_OLD = 5,
            ASENSING_YAW_INCR = 6,
        };

    public:
        SensorHandle(const int imu_type, const std::string imu_dev = "/dev/ttyUSB0", const float imu_frequency = 200);
        ~SensorHandle() = default;
        void setWheelParamters(const double distance_per_left_encoder, const double distance_per_right_encoder,
                               const double wheel_width);

        ImuData getImuWithIdx(int index = 0) const;
        ImuData getImuWithStamp(const uint64_t stamp) const;
        void setStandstill(const bool standstill);
        void setExtTransform(const double roll, const double pitch, const double yaw,
                             const double trans_x, const double trans_y, const double trans_z);
        void setBagParam(const std::string &bag_dir, const std::string &bag_file);

        // std::optional<std::pair<ImuData, OdomData>> getRawDatas();

    private:
        bool imuInit(const ImuType &imu_type, const std::string imu_dev);
        void onImuRawMsg(ImuData &imu);
        void pushImuData(const ImuData &imu);
        bool imuPreprocess(ImuData &imu);
        void updateBias();
        void integrateAngle(Eigen::Quaternionf &last_quat, Eigen::Vector3f &last_w, Eigen::Vector3f &curr_w,
                            Eigen::Quaternionf &curr_quat, double delta_t);
        Eigen::Quaternionf getQuatDeriv(const Eigen::Quaternionf &last_q, Eigen::Vector3f &omega);
        Eigen::Quaternionf integQuqt(const Eigen::Quaternionf &last_q, Eigen::Quaternionf &q_deriv,
                                     double delta_time, double ratio);
        void integrateAngleOptimalSample(Eigen::Quaternionf &last_quat, Eigen::Vector3f &last_w, Eigen::Vector3f &curr_w,
                                         Eigen::Quaternionf &curr_quat, double delta_t);
        bool imuFluctuation(const ImuData &imu);
        bool isEnoughTime(const std::vector<ImuData> &cache_window, uint64_t &ref_time, const uint64_t threshold_time);
        std::shared_ptr<ImuData> pushFluctuationData(const ImuData &imu, const size_t size = 40);

    private:
        int imu_data_size_;
        float imu_ratio_;
        bool is_set_transform_;
        bool is_set_bag_param;

        std::string bag_dir_;
        std::string bag_file_;

        ImuType imu_type_;
        Eigen::Quaternionf ext_rot_; // base -> imu
        Eigen::Translation3f ext_trans_;
        std::shared_ptr<driver::DriverBase> imu_driver_{nullptr};
        
        //imu preprocess
        bool first_process_ = true;
        std::atomic<bool> standstill_;
        Eigen::Quaternionf rotate_q_;
        Eigen::Vector3f last_w_;

        Eigen::Vector6f fluctuation_threshold_;
        Eigen::Vector6f fluctuation_mean_;
        Eigen::Vector6f fluctuation_squar_sum_;
        Eigen::Vector6f bias_init_;
        int fluctuation_size_;
        std::deque<ImuData> fluctuation_data_;
        std::vector<ImuData> cache_window_;
        std::shared_ptr<sros::device::Device> imu_device_;

        const int imu_max_stamp_cache_size_ = 200;
        const int imu_max_buffer_size = 4000;

        //zmy TODO: 改为多线程版本 circle priority queue
        // std::shared_ptr<std::multiset<ImuData, std::greater<ImuData>>> imu_datas_ = std::make_shared<std::multiset<ImuData, std::greater<ImuData>>>();

        std::shared_ptr<ImuCircleArray<ImuData>> imu_datas_;
        // CircleStampArray<int64_t> imu_delta_time_stamp_;
        circle::CircleOptimizerArray<int64_t> imu_delta_time_stamp_;
        std::shared_ptr<sros::MotionStateDetector> state_detector_;
    };
} // namespace imu

#endif