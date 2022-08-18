/**
 * @file sensor_handle.cpp
 * @author zmy (626670628@qq.com)
 * @brief 用于odom，imu融合前的数据获取及处理
 * @version 0.1
 * @date 2021-03-31
 *
 *
 */

#include "sensor_handle.h"
#include "asensing/asensing_driver.h"
#include "atom/atom_driver.h"
// #include "bag/bag_driver.h"
#include "core/device/device_manager.h"
#include "core/util/time.h"
#include "lins/lins_driver.h"
#include <Eigen/Dense>
// #include <optional>

namespace imu
{

    SensorHandle::SensorHandle(const int imu_type, const std::string imu_dev, const float imu_frequency)
        : is_set_transform_(false),
          standstill_(false),
          imu_ratio_(1e6 / imu_frequency),
          imu_data_size_(4000), // 20s的数据
          fluctuation_mean_(Eigen::Vector6f::Zero()),
          fluctuation_squar_sum_(Eigen::Vector6f::Zero()),
          fluctuation_size_(40),
          bias_init_(Eigen::Vector6f::Zero()),
          cache_window_(std::vector<ImuData>()),
          imu_type_(ImuType(imu_type))
    {
        if (!imuInit(imu_type_, imu_dev))
        {
            if (imu_type_ != ImuType::NONE)
                LOG(WARNING) << "IMU init failed !!!\n"
                             << "current imu type is " << imu_type;
        }
        imu_datas_ = std::make_shared<ImuCircleArray<ImuData>>(imu_max_buffer_size);
        state_detector_ = sros::MotionStateDetector::getInstance();
        fluctuation_threshold_ << 4.75e-4, 4.75e-4, 4.75e-4, 9.75e-2, 9.75e-2, 9.75e-2;
    }

    bool SensorHandle::imuInit(const ImuType &imu_type, const std::string imu_dev)
    {
        switch (imu_type)
        {
            // case ImuType::ATOM:
            //     imu_driver_ = std::make_unique<driver::AtomDriver>();
            //     break;

            // case ImuType::LINS:
            //     imu_driver_ = std::make_unique<driver::LinsDriver>();
            //     break;

            // case ImuType::ASENSING:
            //     imu_driver_ = std::make_unique<driver::AsensingDriver>();
            //     break;

        case ImuType::ATOM:
            imu_driver_.reset(new driver::AtomDriver("IMU_ATOM", imu_dev));
            break;

        case ImuType::LINS:
            imu_driver_.reset(new driver::LinsDriver("IMU_LINS", imu_dev));
            break;

        case ImuType::ASENSING:
            imu_driver_.reset(new driver::AsensingDriver("IMU_ASENSING", 0, imu_dev));
            break;

        case ImuType::ASENSING_OLD:
            imu_driver_.reset(new driver::AsensingDriver("IMU_ASENSING_OLD", 1, imu_dev));
            break;

        case ImuType::ASENSING_YAW_INCR:
            imu_driver_.reset(new driver::AsensingDriver("IMU_ASENSING_YAW_INCR", 2, imu_dev));
            break;

        case ImuType::NONE:
            return false;

            // case ImuType::BAG:
            //     if (!is_set_bag_param)
            //     {
            //         LOG(WARNING) << "please set bag param!!!!";
            //         break;
            //     }
            //     imu_driver_.reset(new driver::BagDriver(bag_dir_, bag_file_));
            //     break;

        default:
            break;
        }

        if (imu_driver_ == nullptr)
        {
            LOG(WARNING) << "imu init failed !!!";
            return false;
        }
        sros::device::DeviceManager::getInstance()->addDevice(imu_driver_);

        imu_driver_->setCallback([this](ImuData &data)
                                 { onImuRawMsg(data); });

        if (!imu_driver_->open())
        {
            LOG(WARNING) << "imu init failed !!!";
            return false;
        }

        LOG(INFO) << "successfully to open!";
        return true;
    }

    void SensorHandle::onImuRawMsg(ImuData &imu)
    {
        if (!imuPreprocess(imu))
            return;

        if (!is_set_transform_)
        {
            return;
        }
        // LOG(INFO) << "get Imu raw msg!";
        int64_t imu_timestamp = imu.header.stamp; //us
        static int64_t last_imu_timestamp = 0;    //防止出现雷达掉电,导致时间戳不连续的情况.   us
        if (last_imu_timestamp > imu_timestamp)
        {
            LOG(WARNING) << "imu stamp is wrong!will clear stamp array!";
            imu_delta_time_stamp_.resize(imu_max_stamp_cache_size_);
        }
        last_imu_timestamp = imu_timestamp;
        if (imu_delta_time_stamp_.empty())
        {
            imu_delta_time_stamp_.resize(imu_max_stamp_cache_size_);
        }
        // 时间同步策略
        int64_t delta_time_stamp = imu.header.sync_stamp - imu_timestamp; //us
        imu_delta_time_stamp_.push_back(delta_time_stamp);
        auto min_delta_stamp = imu_delta_time_stamp_.getMinValue(); //us

        int64_t curr_real_time = sros::core::util::get_time_in_us();
        if ((curr_real_time - (imu_timestamp + min_delta_stamp)) > 2e4)
        {
            // 开启了本地调试，IMU数据不对，但是不让其刷屏
            // auto &s = sros::core::Settings::getInstance();
            // auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") ==
            //"True");
            // if (!enable_sros_native_debug) {
            // LOG(WARNING) << "IMU curr delta time is large!"
            //              << ((curr_real_time - (imu_timestamp + min_delta_stamp))) / 1.e6;
            // }
            imu_delta_time_stamp_.resize(imu_max_stamp_cache_size_);
        }

        static Eigen::Quaternionf rectify_quat = Eigen::Quaternionf::Identity();
        auto transformed_imu = imu;
        transformed_imu.angular_velocity = ext_rot_.toRotationMatrix() * imu.angular_velocity;
        transformed_imu.linear_acceleration = ext_rot_.toRotationMatrix() * imu.linear_acceleration;
        transformed_imu.orientation = ext_rot_.inverse() * imu.orientation * ext_rot_ * rectify_quat;
        transformed_imu.header.stamp = min_delta_stamp + imu.header.stamp;
        transformed_imu.status = 1;

        static std::pair<ImuData, uint64_t> last_data = std::make_pair(transformed_imu, 0);

        if (transformed_imu.orientation.angularDistance(last_data.first.orientation) > 0.02) //两帧间角度大于1.7° 进行校正
        {
            LOG(WARNING) << "IMU Quaternion diff angleeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee...:"
                         << (transformed_imu.orientation.angularDistance(last_data.first.orientation));

            // LOG(INFO) << transformed_imu.angular_velocity;

            // LOG(WARNING) << "IMU Quaternion abrupt change，using last imu data...";

            // auto &last_imu = last_data.first;
            // //zmy XXX:使用了积分前的差值，使用积分后是否更好
            // rectify_quat = transformed_imu.orientation.inverse() * last_imu.orientation * rectify_quat;
            // auto half_delta = static_cast<double>(transformed_imu.header.stamp - last_imu.header.stamp) / 2e6;
            // last_imu.orientation *= Eigen::Quaternionf(1, last_imu.angular_velocity.x() * half_delta,
            //                                            last_imu.angular_velocity.y() * half_delta,
            //                                            last_imu.angular_velocity.z() * half_delta);
            // last_imu.header.stamp = transformed_imu.header.stamp;
            // last_imu.status = 2;

            auto check_delta = transformed_imu.header.stamp - last_data.second;
            last_data.second = check_delta == transformed_imu.header.stamp
                                   ? transformed_imu.header.stamp
                                   : last_data.second;
            if (check_delta > (imu_ratio_ / 2) && check_delta != transformed_imu.header.stamp)
            {
                last_data.first = transformed_imu;
                LOG(WARNING) << "IMU Quaternion deviates from a reasonable value for " << check_delta << " us!!!";
                return;
            }
            else
            {
                LOG(WARNING) << "IMU Quaternion abrupt change，using last imu data...";
                auto &last_imu = last_data.first;
                //zmy XXX:使用了积分前的差值，使用积分后是否更好
                // rectify_quat = transformed_imu.orientation.inverse() * last_imu.orientation * rectify_quat;
                auto half_delta = static_cast<double>(transformed_imu.header.stamp - last_imu.header.stamp) / 2e6;
                last_imu.orientation *= Eigen::Quaternionf(1, last_imu.angular_velocity.x() * half_delta,
                                                           last_imu.angular_velocity.y() * half_delta,
                                                           last_imu.angular_velocity.z() * half_delta);
                last_imu.header.stamp = transformed_imu.header.stamp;
                last_imu.status = 2;

                pushImuData(last_imu);
                return;
            }

            // pushImuData(last_imu);
            // return;
        }

        last_data.first = transformed_imu;
        last_data.second = 0;

        pushImuData(std::move(transformed_imu));
        // auto non_const_imu = const_cast<ImuData *>(&imu);
        // non_const_imu->header.stamp = min_delta_stamp + imu.header.stamp;
        // non_const_imu->status = true;
        // const_cast<ImuData *>(&imu)->header.stamp = min_delta_stamp + imu.header.stamp;
        // pushImuData(std::move(imu));
        // timed_imu_data.yaw = static_cast<double>(imu.euler[2]) * 3.1415926535897 / 180.;
        // XXX: 注意这里要使用const_cast后的数据，原始的imu数据时间没有改变
        // pushImuData(std::move(*non_const_imu));
    }

    // std::optional<std::pair<ImuData, OdomData>> SensorHandle::getRawDatas()
    // {
    //     auto current_stamp = sros::core::util::get_time_in_ms();

    //     if (odom_datas_->size() < 2 || imu_datas_->size() < 2)
    //     {
    //         return std::nullopt;
    //     }

    //     // 联通SRC后回复此处
    //     /*******************************************************************************
    //     assert(current_stamp >= imu_datas_->cbegin()->header.stamp && current_stamp >= odom_datas_->begin()->header.stamp);
    //     auto nh = odom_datas_->extract(odom_datas_->cbegin());
    //     nh.value().twist.linear = (odom_datas_->begin()->pose.position - (++odom_datas_->begin())->pose.position) /
    //                               (odom_datas_->begin()->header.stamp - (++odom_datas_->begin())->header.stamp) * 1e6;
    //     odom_datas_->insert(std::move(nh));
    //     *******************************************************************************/
    //     return std::make_optional<std::pair<ImuData, OdomData>>(std::make_pair(*imu_datas_->begin(), *odom_datas_->begin()));
    // }

    void SensorHandle::pushImuData(const ImuData &imu)
    {
        imu_datas_->push_back(imu);
    }

    ImuData SensorHandle::getImuWithIdx(int index) const
    {
        if (imu_datas_->empty()) {
            return ImuData{};
        }else{
            return imu_datas_->array(index);
        }
    }

    ImuData SensorHandle::getImuWithStamp(const uint64_t stamp) const
    {
        if (imu_datas_->size()<4) {
            if (imu_type_ != ImuType::NONE) {
                LOG_EVERY_N(WARNING, 100) << "without correspond imu data to odom stamp";
            }
            return ImuData{};
        }
        auto &datas = *imu_datas_;
        auto data_size = datas.size();
        auto start_stamp = datas[1].header.stamp;
        auto end_stamp = datas[data_size - 2].header.stamp;
        double max_sample_stamp = 10 * imu_ratio_;
        double max_adjcent_delta_stamp = 2e4;//20ms
        double max_delta_stamp = 6e4;//60ms
        int search_array_size = 6;
        // LOG(INFO) << "stamp: " << stamp << " start_stamp: " << start_stamp << " end_stamp: " << end_stamp;

        if (stamp >= start_stamp && stamp <= (end_stamp + max_sample_stamp)) {
            int delta_index = (data_size - 2)/(double(end_stamp - start_stamp)) * (stamp - start_stamp);
            // LOG(INFO) << "进入查找 delta_index： " << delta_index << " data_size: " << data_size;
            if (delta_index >= data_size) {
                delta_index = data_size - 1;
            }
            if (delta_index < 0) {
                delta_index = 0;
            }
            if (datas[delta_index].header.stamp >= stamp + max_adjcent_delta_stamp ||
                datas[delta_index].header.stamp <= stamp - max_adjcent_delta_stamp) {//找到了相近的imu时间戳
                auto min_index = 0;
                auto max_index = data_size;
                auto mid_index = 0;
                int count = 0;
                // LOG(INFO) << "二分查找！！！！";
                while (max_index - min_index > 1) {
                    mid_index = (min_index + max_index) / 2;
                    if (stamp > datas[mid_index].header.stamp) {
                        min_index = mid_index;
                    }
                    else if (stamp < datas[mid_index].header.stamp) {
                        max_index = mid_index;
                    } else {
                        delta_index = mid_index;
                        // LOG(INFO) << "二分法找到： " << mid_index;
                        break;
                    }
                    count++;
                    if (count > 30) {
                        LOG(INFO) << "循环死掉！！";
                        break;
                    }
                    // LOG(INFO) << "max: " << max_index << " min: " << min_index << " mid: " << mid_index << " count: " << count;
                }
            }

            std::vector<ImuData> match_imu_data;
            for (int i = 0; i < search_array_size; ++i) {
                int first_index = delta_index - i;
                int second_index = delta_index + i;
                if (first_index >= 0) {
                    auto &curr_imu = datas[first_index];
                    if(abs(double(curr_imu.header.stamp) - double(stamp))<max_delta_stamp){
                        match_imu_data.push_back(curr_imu);
                    }
                }
                if (second_index < data_size) {
                    auto &curr_imu = datas[second_index];
                    if(abs(double(curr_imu.header.stamp) - double(stamp))<max_delta_stamp){
                        match_imu_data.push_back(curr_imu);
                    }
                }
                if (match_imu_data.size() >= search_array_size) {
                    break;
                }
            }

            if (match_imu_data.empty()) {
                LOG(INFO) << "空的！！";
                return ImuData{};
            } else {
                auto result_data = match_imu_data[0];

                std::vector<double> delta_time_member;
                double total_delta_time = 0.0;
                for (int i = 0; i < match_imu_data.size(); i++) {
                    double temp = 0.0;
                    if (stamp >= match_imu_data[i].header.stamp) {
                        temp = static_cast<double> (stamp - match_imu_data[i].header.stamp) * 1e-6;
                    } else {
                        temp = -static_cast<double> (match_imu_data[i].header.stamp - stamp) * 1e-6;
                    }
                    delta_time_member.push_back(temp);
                    total_delta_time += std::abs(temp);     
                    // LOG(INFO) << "temp time: " << temp;         
                }

                // LOG(INFO) << "total time: " <<  total_delta_time;

                double delta_t = delta_time_member[0];
                Eigen::Quaternionf orientation_chosed_imu = match_imu_data[0].orientation * 
                                                            Eigen::Quaternionf(1, match_imu_data[0].angular_velocity.x() * delta_t,
                                                                                match_imu_data[0].angular_velocity.y() * delta_t,
                                                                                match_imu_data[0].angular_velocity.z() * delta_t);
                result_data.orientation = orientation_chosed_imu;  // 使用chosed_imu中第一个数据初始化result_data
                result_data.orientation.normalize(); 

                for (int i = 1; i < match_imu_data.size(); i++) {
                    delta_t = delta_time_member[i];
                    Eigen::Quaternionf orientation_chosed_imu = match_imu_data[i].orientation * 
                                                                Eigen::Quaternionf(1, match_imu_data[i].angular_velocity.x() * delta_t,
                                                                                    match_imu_data[i].angular_velocity.y() * delta_t,
                                                                                    match_imu_data[i].angular_velocity.z() * delta_t);
                    delta_t = delta_t < 0.0? -delta_t : delta_t;                                                                                                             
                    double ratio = 1.0 - delta_t / total_delta_time;
                    Eigen::Quaternionf compose_q = result_data.orientation.slerp(ratio, orientation_chosed_imu);
                    result_data.orientation = compose_q;
                    result_data.orientation.normalize();                                                                                                            
                }
                return result_data;                    
            }
            
        } else {
            LOG(WARNING) << "cannot find correspond imu data!";
            LOG(INFO) << "stamp: " << stamp << " start_stamp: " << start_stamp << " end_stamp: " << end_stamp;
            return ImuData{};
        }
    }

    void SensorHandle::setExtTransform(const double roll, const double pitch, const double yaw,
                                       const double trans_x, const double trans_y, const double trans_z)
    {
        LOG(INFO) << "IMU extren transform: roll " << roll << " pitch " << pitch << " yaw " << yaw << "\n"
                  << "translate_x " << trans_x << " translate_y " << trans_y << " translate_z " << trans_z;
        ext_rot_ = Eigen::Quaternionf(Eigen::AngleAxisf(roll / 180 * M_PI, Eigen::Vector3f::UnitX()) *
                                      Eigen::AngleAxisf(pitch / 180 * M_PI, Eigen::Vector3f::UnitY()) *
                                      Eigen::AngleAxisf(yaw / 180 * M_PI, Eigen::Vector3f::UnitZ()));
        ext_trans_ = Eigen::Translation3f(trans_x, trans_y, trans_z);
        is_set_transform_ = true;
    }

    void SensorHandle::setBagParam(const std::string &bag_dir, const std::string &bag_file)
    {
        bag_dir_ = bag_dir;
        bag_file_ = bag_file_;
        is_set_bag_param = true;
    }

    bool SensorHandle::imuPreprocess(ImuData &imu)
    {
        static bool starting_flag = true;
        static uint64_t ref_stamp = imu.header.stamp;
        static uint64_t last_process_time = 0;

        if (starting_flag)
        { //zmy: 是否会有时间异常
            if (imu.header.stamp - ref_stamp < 1e5)
                return false;
            else
            {
                starting_flag = false;
                ref_stamp = 0;
            }
        }

        if (state_detector_->staticForLongDuration(2e6) && (imu.header.stamp - last_process_time > 3.6e8 || bias_init_.isZero()) &&
            (imu.angular_velocity - bias_init_.head<3>()).squaredNorm() < 0.01)
        {
            if (imuFluctuation(imu) && !isEnoughTime(cache_window_, ref_stamp, 1e7))
            {
                LOG_IF(INFO, cache_window_.empty()) << "Start caculating IMU bias...";
                cache_window_.emplace_back(imu);
                // return false;
            }
            else if (cache_window_.size() > fluctuation_size_ && cache_window_.back().header.stamp - cache_window_.front().header.stamp > 7e5)
            {
                updateBias();
                cache_window_.clear();
                fluctuation_data_.clear();
                fluctuation_mean_.setZero();
                fluctuation_squar_sum_.setZero();
                ref_stamp = 0;
                last_process_time = imu.header.stamp;
            }
        }
        else if (!cache_window_.empty() || !fluctuation_data_.empty())
        {
            if (cache_window_.size() > fluctuation_size_ &&
                cache_window_.back().header.stamp - cache_window_.front().header.stamp > 7e5)
            {
                updateBias();
                last_process_time = imu.header.stamp;
            }
            cache_window_.clear();
            fluctuation_data_.clear();
            fluctuation_mean_.setZero();
            fluctuation_squar_sum_.setZero();
            ref_stamp = 0;
        }

        const Eigen::Vector3f &acc_v = imu.linear_acceleration;

        if (first_process_)
        {
            first_process_ = false;
            tf::EulerToEigenQuaternion(0.0, 0.0,
                                       0.0, rotate_q_);
            last_w_ = imu.angular_velocity - bias_init_.head<3>();
        }
        else
        {
            double delta_time = 0.005; //zmy:TODO 改为传参
            Eigen::Vector3f omega = imu.angular_velocity - bias_init_.head<3>();
            Eigen::Quaternionf next_q, deriv_q;

            //            integrateAngle(rotate_q_, last_w_, omega, next_q, delta_time);
            integrateAngleOptimalSample(rotate_q_, last_w_, omega, next_q, delta_time); //shj: 优化单子样算法

            rotate_q_ = next_q;
            last_w_ = omega;
            double gyro_scale = 100 * M_PI / 180.0;
            if (gyro_scale < omega.norm())
            {
                LOG(INFO) << "原始角速度:" << omega[0] << "," << omega[1] << ","
                          << omega[2] << ",角速度最大值，" << 250 * M_PI / 180.0 << ",采样的十六位值：" << imu.angular_velocity.z();
            }

            Eigen::Vector3f predict_acc = rotate_q_ * acc_v;
            predict_acc.normalize();

            //注意此处vector3f.z()的符号，依据imu坐标系z轴朝向调整
            auto weight_q = Eigen::Quaternionf::FromTwoVectors(predict_acc, Eigen::Vector3f(0, 0, -1));
            auto theta = acos(weight_q.w());
            if (fabs(theta) > 0.001)
            {
                auto weight_value = 0.005;
                double acc_v_norm = acc_v.norm();
                auto acc_ratio = acc_v_norm / 9.7887;
                weight_value /= pow(acc_ratio, 4);
                // LOG(INFO) << "weight value:" << weight_value;
                bool need_fusion = false;
                if (acc_v.norm() > 10.3f || acc_v.norm() < 9.3f)
                {
                    weight_q.setIdentity();
                }
                else
                {
                    auto new_theta = theta;
                    if (weight_value <= 1.0)
                    {
                        new_theta *= weight_value; //用来乘以权重，减小角度影响
                    }
                    // LOG(INFO) << "theta:" << new_theta;
                    if (cos(theta) != 0 && sin(theta) != 0)
                    {
                        need_fusion = true;
                        weight_q.w() = weight_q.w() / cos(theta) * cos(new_theta);
                        weight_q.x() = weight_q.x() / sin(theta) * sin(new_theta);
                        weight_q.y() = weight_q.y() / sin(theta) * sin(new_theta);
                        weight_q.z() = weight_q.z() / sin(theta) * sin(new_theta);
                        weight_q.normalize();
                        // LOG(INFO) << "curr data:" << acos(weight_q.w());
                    }
                    else
                    {
                        weight_q.setIdentity();
                    }
                }
                if (need_fusion)
                {
                    Eigen::Quaternionf quat_temp = weight_q * rotate_q_;
                    rotate_q_ = quat_temp;
                }
            }

            imu.orientation.w() = rotate_q_.w();
            imu.orientation.x() = rotate_q_.x();
            imu.orientation.y() = rotate_q_.y();
            imu.orientation.z() = rotate_q_.z();

            imu.angular_velocity -= bias_init_.head<3>();

            return true;
        }

        return false;
    }

    /* 优化单字样旋转矢量法
         * phi = dtheta1 + 1/12 *  (dtheta1 + dtheta0)
        */
    void SensorHandle::integrateAngleOptimalSample(Eigen::Quaternionf &last_quat, Eigen::Vector3f &last_w, Eigen::Vector3f &curr_w,
                                                   Eigen::Quaternionf &curr_quat, double delta_t)
    {
        Eigen::Vector3f phi = curr_w * delta_t + (last_w * delta_t).cross(curr_w * delta_t) / 12.0;
        float norm_phi = phi.norm();
        if (norm_phi < 1e-9)
        {
            norm_phi = 0;
            phi.setZero();
        }
        else
        {
            phi /= norm_phi;
        }

        Eigen::AngleAxisf rotation_vector(norm_phi, phi);
        Eigen::Quaternionf dq(rotation_vector);
        curr_quat = (last_quat * dq).normalized();
    }

    void SensorHandle::integrateAngle(Eigen::Quaternionf &last_quat, Eigen::Vector3f &last_w, Eigen::Vector3f &curr_w,
                                      Eigen::Quaternionf &curr_quat, double delta_t)
    {
        Eigen::Vector3f mid_w = (0.5) * (last_w + curr_w);
        Eigen::Quaternionf k1, k2, k3, k4, tmp_q;
        k1 = getQuatDeriv(last_quat, last_w);
        tmp_q = integQuqt(last_quat, k1, delta_t, 0.5);
        k2 = getQuatDeriv(tmp_q, mid_w);
        tmp_q = integQuqt(last_quat, k2, delta_t, 0.5);
        k3 = getQuatDeriv(tmp_q, mid_w);
        tmp_q = integQuqt(last_quat, k3, delta_t, 1.0);
        k4 = getQuatDeriv(tmp_q, curr_w);
        double ratio_1 = 1 / 6.0;
        double ratio_2 = 1 / 3.0;
        curr_quat = integQuqt(last_quat, k1, delta_t, ratio_1);
        curr_quat = integQuqt(curr_quat, k2, delta_t, ratio_2);
        curr_quat = integQuqt(curr_quat, k3, delta_t, ratio_2);
        curr_quat = integQuqt(curr_quat, k4, delta_t, ratio_1);
        curr_quat.normalize();
    }

    Eigen::Quaternionf SensorHandle::getQuatDeriv(const Eigen::Quaternionf &last_q, Eigen::Vector3f &omega)
    {
        Eigen::Quaternionf deriv_q;
        deriv_q.w() = (-last_q.x() * omega[0] - last_q.y() * omega[1] - last_q.z() * omega[2]) * 0.5;
        deriv_q.x() = (last_q.w() * omega[0] - last_q.z() * omega[1] + last_q.y() * omega[2]) * 0.5;
        deriv_q.y() = (last_q.z() * omega[0] + last_q.w() * omega[1] - last_q.x() * omega[2]) * 0.5;
        deriv_q.z() = (-last_q.y() * omega[0] + last_q.x() * omega[1] + last_q.w() * omega[2]) * 0.5;
        return deriv_q;
    }

    Eigen::Quaternionf SensorHandle::integQuqt(const Eigen::Quaternionf &last_q, Eigen::Quaternionf &q_deriv, double delta_time, double ratio)
    {
        Eigen::Quaternionf int_q;
        auto real_time = delta_time * ratio;
        int_q.w() = real_time * q_deriv.w() + last_q.w();
        int_q.x() = real_time * q_deriv.x() + last_q.x();
        int_q.y() = real_time * q_deriv.y() + last_q.y();
        int_q.z() = real_time * q_deriv.z() + last_q.z();
        return int_q;
    }

    bool SensorHandle::imuFluctuation(const ImuData &imu)
    {
        if (state_detector_->staticForLongDuration(2e6) &&
            (imu.angular_velocity - bias_init_.head<3>()).squaredNorm() < 0.01)
        {
            auto dat = pushFluctuationData(imu, fluctuation_size_);
            if (dat == nullptr)
            {
                return false;
            }
            else if (fluctuation_mean_.isZero())
            {
                for (const auto &data : fluctuation_data_)
                {
                    fluctuation_mean_.head<3>() += data.angular_velocity;
                    fluctuation_mean_.tail<3>() += data.linear_acceleration;
                }
                fluctuation_mean_ /= fluctuation_data_.size();
                for (const auto &data : fluctuation_data_)
                {
                    fluctuation_squar_sum_.head<3>() += (data.angular_velocity - fluctuation_mean_.head<3>()).array().square().matrix();
                    fluctuation_squar_sum_.tail<3>() += (data.linear_acceleration - fluctuation_mean_.tail<3>()).array().square().matrix();
                }
            }
            else
            {
                fluctuation_squar_sum_.head<3>() -= (dat->angular_velocity - fluctuation_mean_.head<3>()).array().square().matrix();
                fluctuation_squar_sum_.tail<3>() -= (dat->linear_acceleration - fluctuation_mean_.tail<3>()).array().square().matrix();

                fluctuation_mean_.head<3>() += (imu.angular_velocity - dat->angular_velocity) / fluctuation_data_.size();
                fluctuation_mean_.tail<3>() += (imu.linear_acceleration - dat->linear_acceleration) / fluctuation_data_.size();

                fluctuation_squar_sum_.head<3>() += (imu.angular_velocity - fluctuation_mean_.head<3>()).array().square().matrix();
                fluctuation_squar_sum_.tail<3>() += (imu.linear_acceleration - fluctuation_mean_.tail<3>()).array().square().matrix();
            }

            if ((fluctuation_squar_sum_.array() > fluctuation_threshold_.array()).isZero())
                return true;
            else
            {
                fluctuation_mean_.setZero();
                fluctuation_squar_sum_.setZero();
                fluctuation_data_.clear();
            }
        }

        return false;
    }

    std::shared_ptr<ImuData> SensorHandle::pushFluctuationData(const ImuData &imu, const size_t size)
    {
        fluctuation_data_.emplace_back(imu);
        if (fluctuation_data_.size() > size)
        {
            ImuData front_imu = fluctuation_data_.front();
            fluctuation_data_.pop_front();
            return std::make_shared<ImuData>(front_imu);
        }
        return nullptr;
    }

    bool SensorHandle::isEnoughTime(const std::vector<ImuData> &cache_window, uint64_t &ref_time, const uint64_t threshold_time)
    {
        if (cache_window.empty())
            return false;
        else if (ref_time == 0)
        {
            ref_time = cache_window.front().header.stamp;
            return false;
        }
        else
        {
            return cache_window.back().header.stamp - ref_time > threshold_time;
        }
    }

    void SensorHandle::setStandstill(const bool standstill)
    {
        if (standstill_ != standstill)
        {
            standstill_.store(standstill);
        }
    }

    void SensorHandle::updateBias()
    {
        if (cache_window_.size() > fluctuation_size_)
        {
            int bias_cache_size = cache_window_.size() - fluctuation_size_;
            bias_init_.setZero();
            for (int i = 0; i < bias_cache_size; ++i)
            {
                bias_init_.head<3>() += cache_window_[i].angular_velocity;
                bias_init_.tail<3>() += cache_window_[i].linear_acceleration;
            }
            bias_init_ /= bias_cache_size;
            LOG(INFO) << "Finshed imu bias caculate!!!";
            LOG(INFO) << "bias: " << bias_init_[2] * 180/3.14;
        }
    }

} // namespace imu
