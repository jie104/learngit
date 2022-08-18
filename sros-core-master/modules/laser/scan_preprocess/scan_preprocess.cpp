/**
 * @file scan_preprocess.cpp
 * @author zmy (626670628@qq.com)
 * @brief laser运动补偿实现
 * @version 0.1
 * @date 2021-06-18
 * 
 * 
 */

#include "scan_preprocess.hpp"
#include <cmath>

namespace laser
{
    ScanPreprocess::ScanPreprocess(slam::tf::TFOperator *tf_base_to_odo)
        : correct_type_(DistortionCorrectType::ODOM),
          laser_quat_(Eigen::Quaternionf::Identity()),
          laser_trans_(Eigen::Vector3f::Zero()),
          last_scan_cache_num_(100),
          angle_max_(M_PI),
          angle_min_(-M_PI),
          last_back_ranges_(std::vector<std::tuple<float, float, float>>()),tf_base_to_odo_(tf_base_to_odo)
    {
    }

    void ScanPreprocess::correctScan(sros::core::LaserScan_ptr scan_ptr,  const bool is_upside)
    {
        // todo： 还需要严格验证计算时间，尽可能保证精度，考虑浮点数计算误差，扫描出现不完整等情况. 避免计算PI这样的浮点数，直接用number。
        int64_t scan_time = scan_ptr->scan_time * 1e6; /// !! should transform time in s to us.
        int64_t valid_scan_time = (scan_ptr->angle_max - scan_ptr->angle_min) / (2*M_PI) * scan_time;
        int64_t scan_begin_time = static_cast<uint64_t>(scan_ptr->time_ - valid_scan_time/2);
        int64_t scan_end_time = scan_begin_time + valid_scan_time;
        std::vector<slam::tf::TransForm> tfs;
        tfs.reserve(10);
        tf_base_to_odo_->lookUpTransForms(scan_begin_time, scan_end_time, tfs);
        if (tfs.size() < 2) {
            return;
        }

        bool is_descend = std::signbit(scan_ptr->time_increment);   // 万集雷达电机反转情况
        double time_increment = std::fabs(scan_ptr->time_increment);
        bool is_reverse = is_upside == is_descend;
        double angle = !is_reverse ? scan_ptr->angle_min : scan_ptr->angle_max;
        float angle_max = angle_max_;
        float angle_min = angle_min_;
        std::vector<std::tuple<float, float, float>> undistorted_points;
        undistorted_points.reserve(scan_ptr->ranges.size());
        std::vector<std::tuple<float, float, float>> ranges_cache;
        size_t batch_idx = 0;
        float batch_sum = 0;

        int last_idx = scan_ptr->ranges.size() - 1;
        int cache_threshold = !is_reverse ? last_idx - last_scan_cache_num_ : last_scan_cache_num_;

        const Eigen::Quaternionf &laser_tf_quat = laser_quat_;
        Eigen::Quaternionf laser_tf_quat_inv = laser_tf_quat.inverse();
        const Eigen::Vector3f &laser_tf_trans = laser_trans_;

        if (correct_type_ == DistortionCorrectType::ODOM)
        {
            Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
            Eigen::Vector3f delta_pos = Eigen::Vector3f::Zero();
            Eigen::Quaternionf delta_quat = Eigen::Quaternionf::Identity();
            Eigen::Quaternionf ref_quat = Eigen::Quaternionf::Identity();
            Eigen::Vector3f ref_pos = Eigen::Vector3f::Zero();
            size_t ref_idx = 0;
            float delta_idx = 1;
            auto tf_size = tfs.size();
            for (size_t i = 0; i < scan_ptr->ranges.size(); ++i)
            {

                if (i >= batch_sum)
                {
                    if (batch_idx == 0)
                    {
                        auto &batch_id_tf = tfs[batch_idx];
                        auto &batch_id_next_tf = tfs[batch_idx+1];
                        Eigen::Quaterniond batch_id_q, batch_id_next_q;
                        batch_id_tf.getQuaternion(batch_id_q);
                        batch_id_next_tf.getQuaternion(batch_id_next_q);
                        Eigen::Quaternionf delta_q =
                            (batch_id_q.cast<float>().inverse() * batch_id_next_q.cast<float>()).normalized(); // 前后两个位姿的角度差

                        batch_sum = static_cast<double>(batch_id_next_tf.pose_time - scan_begin_time) /
                                    (time_increment * 1.0e6);

                        float first_id = static_cast<double>(batch_id_tf.pose_time - scan_begin_time) /
                                         (time_increment * 1.0e6);

                        delta_idx = batch_sum - ref_idx;
                        float ratio = first_id / (batch_sum - first_id);
                        delta_quat = identity.slerp(ratio, delta_q) * delta_q;
                        Eigen::Vector3f delta_point_float = Eigen::Vector3f(batch_id_next_tf.position.x() - batch_id_tf.position.x(),
                                                             batch_id_next_tf.position.y() - batch_id_tf.position.y(),
                                                             batch_id_next_tf.position.z() - batch_id_tf.position.z());
                        delta_pos = (batch_id_q.cast<float>().inverse() * delta_point_float * (1 + ratio));
                    }
                    else if (batch_idx + 1 == tf_size)
                    {
                        ref_pos = ref_quat * delta_pos + ref_pos;
                        ref_quat = ref_quat * delta_quat;
                        ref_idx = batch_sum;
                        batch_sum = scan_ptr->ranges.size();
                        float ratio = (batch_sum - ref_idx) / delta_idx;
                        delta_idx = batch_sum - ref_idx;
                        delta_quat = identity.slerp(ratio, delta_quat);
                        delta_pos = delta_pos * ratio;
                    }
                    else
                    {
                        auto &batch_id_tf = tfs[batch_idx];
                        auto &batch_id_next_tf = tfs[batch_idx + 1];

                        ref_pos = ref_quat * delta_pos + ref_pos;
                        ref_quat = ref_quat * delta_quat;
                        Eigen::Quaterniond batch_id_q, batch_id_next_q;
                        batch_id_tf.getQuaternion(batch_id_q);
                        batch_id_next_tf.getQuaternion(batch_id_next_q);
                        ref_idx = batch_sum;
                        batch_sum = static_cast<double>(batch_id_next_tf.pose_time - scan_begin_time) /
                                    (time_increment * 1e6);

                        delta_idx = batch_sum - ref_idx;
                        Eigen::Quaternionf delta_q =
                            (batch_id_q.cast<float>().inverse() * batch_id_next_q.cast<float>()).normalized();
                        Eigen::Vector3f delta_point_float = Eigen::Vector3d(batch_id_next_tf.position.x() - batch_id_tf.position.x(),
                                                                            batch_id_next_tf.position.y() - batch_id_tf.position.y(),
                                                                            batch_id_next_tf.position.z() - batch_id_tf.position.z()).cast<float>();
                        delta_pos = batch_id_q.cast<float>().inverse() * delta_point_float;
                    }

                    ++batch_idx;
                }

                int point_idx = !is_reverse ? i : last_idx - i;
                if (scan_ptr->ranges[point_idx] <= scan_ptr->range_max &&
                    scan_ptr->ranges[point_idx] >= scan_ptr->range_min &&
                    angle <= angle_max && angle >= angle_min)
                {
                    auto pos = toPos(scan_ptr->ranges[point_idx], angle);

                    float ratio = (i - ref_idx) / delta_idx;
                    Eigen::Quaternionf local_delta_quat = ref_quat * identity.slerp(ratio, delta_quat);
                    Eigen::Vector3f delta_trans = ref_quat * (ratio * delta_pos) + ref_pos;
                    auto cor_pos = laser_tf_quat_inv * (local_delta_quat * laser_tf_quat * pos +
                                                        (local_delta_quat * laser_tf_trans + delta_trans - laser_tf_trans));
                    undistorted_points.emplace_back(cor_pos.norm(), std::atan2(cor_pos.y(), cor_pos.x()), scan_ptr->intensities[point_idx]);

                    bool flag = !is_reverse ? point_idx > cache_threshold : point_idx < cache_threshold;
                    if (flag)
                    {
                        float cache_angle = !is_reverse ? scan_ptr->angle_max - (last_idx - point_idx) * scan_ptr->angle_increment
                                                        : scan_ptr->angle_min + point_idx * scan_ptr->angle_increment;
                        ranges_cache.emplace_back(scan_ptr->ranges[point_idx], cache_angle, scan_ptr->intensities[point_idx]);
                    }
                }

                angle = !is_reverse ? angle + scan_ptr->angle_increment : angle - scan_ptr->angle_increment;
            }
        }
        else
        {
            LOG(WARNING) << "Laser distortion correct type is not a reasonably setting ,please check it !!";
            return;
        }

        undistorted_points.insert(undistorted_points.end(), last_back_ranges_.begin(), last_back_ranges_.end());
        std::sort(undistorted_points.begin(), undistorted_points.end(),
                  [](const std::tuple<float, float, float> &lhs, const std::tuple<float, float, float> &rhs)
                  { return std::get<1>(lhs) < std::get<1>(rhs); });
        toLaserScan(undistorted_points, scan_ptr);
        last_back_ranges_.swap(ranges_cache);
        return;
    }

    // 极坐标系到直角坐标系的转换
    Eigen::Vector3f ScanPreprocess::toPos(const float range, const double angle) const
    {
        return Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()) * (range * Eigen::Vector3f::UnitX());
    }

    Eigen::Vector3f ScanPreprocess::extrapolatePoint(const Eigen::Vector3f &pos, const double delta_time,
                                                     const Eigen::Vector3f &linear, const Eigen::Vector3f &angular,
                                                     Eigen::Vector3f &newest_pos, Eigen::Quaternionf &newest_quat) const
    {
        Eigen::Vector3f delta_angular = 0.5 * angular * delta_time;
        newest_quat = (newest_quat * Eigen::Quaternionf(1, delta_angular[0], delta_angular[1], delta_angular[2])).normalized();
        newest_pos += newest_quat * (linear * delta_time);
        return newest_quat * pos + newest_pos;
    }

    void ScanPreprocess::toLaserScan(const std::vector<std::tuple<float, float, float>> &undistorted_points,
                                     sros::core::LaserScan_ptr scan) const
    {
        const float max_db = 2.f * scan->angle_increment;
        const float min_db = 0.7f * scan->angle_increment;
        float angle = scan->angle_min;
        auto iter = undistorted_points.begin();
        scan->undistorted_ranges.resize(scan->ranges.size());
        scan->undistorted_intensities.resize(scan->intensities.size());
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            while (iter != undistorted_points.end() && std::get<1>(*iter) < angle)
            {
                ++iter;
            }

            scan->undistorted_ranges[i] = 0.f;
            scan->undistorted_intensities[i] = 0.f;
            if (iter != undistorted_points.begin() && iter != undistorted_points.end())
            {
                const float b1 = std::get<1>(*std::prev(iter));
                const float b2 = std::get<1>(*iter);
                const float db = b2 - b1;
                const float r2 = std::get<0>(*iter);
                const float r1 = std::get<0>(*std::prev(iter));
                if (db <= max_db)
                {
                    const float db1 = angle - b1;
                    const float db2 = db - db1;

                    if (r1 != 0.f || r2 != 0.f)
                    {
                        // 分角线性质: https://blog.csdn.net/weixin_29689845/article/details/112162867
                        scan->undistorted_ranges[i] = r1 * r2 * db / (r1 * db1 + r2 * db2);
                        scan->undistorted_intensities[i] = 0.5 * (std::get<2>(*std::prev(iter)) + std::get<2>(*iter));
                    }
                }
                else if ((angle - std::get<1>(*std::prev(iter))) <= min_db)
                {
                    scan->undistorted_ranges[i] = r1 * std::cos(angle - b1);
                    scan->undistorted_intensities[i] = std::get<2>(*std::prev(iter));
                }
                else if ((std::get<1>(*iter) - angle) <= min_db)
                {
                    scan->undistorted_ranges[i] = r2 * std::cos(b2 - angle);
                    scan->undistorted_intensities[i] = std::get<2>(*iter);
                }
                else
                {
                    scan->undistorted_ranges[i] = 0;
                    scan->undistorted_intensities[i] = 0.f;
                }
            }
            else if (iter == undistorted_points.begin() && std::get<1>(*iter) - angle < min_db)
            {
                scan->undistorted_ranges[i] = std::get<0>(*iter) * std::cos(std::get<1>(*iter) - angle);
                scan->undistorted_intensities[i] = std::get<2>(*iter);
            }

            angle += scan->angle_increment;
        }
    }

    Eigen::Affine2f ScanPreprocess:: getDeltaLaserTF(const int64_t stamp, const int64_t delta_time) const
    {
        int64_t end_stamp = stamp;
        if (delta_time < 0) {
            end_stamp -= delta_time;
        }else{
            end_stamp += delta_time;
        }
        std::vector<slam::tf::TransForm> tfs;
        tfs.reserve(10);
        tf_base_to_odo_->lookUpTransForms(stamp, end_stamp,tfs);
        if (tfs.size() < 2) {
            return Eigen::Affine2f::Identity();
        }
        else
        {
            auto& tf_back = tfs.back();
            auto &tf_front = tfs.front();
            Eigen::Quaterniond quat_back,quat_front;
            tf_back.getQuaternion(quat_back);
            tf_front.getQuaternion(quat_front);
            Eigen::Vector3f delta_point = Eigen::Vector3f(tf_front.position.x() - tf_back.position.x(),
                                                          tf_front.position.y() - tf_back.position.y(),
                                                          tf_front.position.z() - tf_back.position.z());

            auto delta_tran = quat_back.cast<float>().inverse() *
                              delta_point;
            auto quaternion = (quat_back.cast<float>().inverse() *quat_front.cast<float>()).normalized();
            if (delta_time < 0) {
                auto delta_tran = quat_front.cast<float>().inverse() *
                                  (-delta_point);
                auto quaternion = (quat_front.cast<float>().inverse() *quat_back.cast<float>()).normalized();
            }

            const Eigen::Quaternionf &laser_tf_quat = laser_quat_;
            const Eigen::Vector3f &laser_tf_trans = laser_trans_;
            Eigen::Quaternionf laser_tf_quat_inv = laser_tf_quat.inverse();
            Eigen::Quaternionf delta_laser_quat = (laser_tf_quat_inv * quaternion * laser_tf_quat).normalized();
            Eigen::Vector3f delta_laser_trans = laser_tf_quat_inv * (quaternion * laser_tf_trans + delta_tran - laser_tf_trans);

            auto w = delta_laser_quat.w();
            auto x = delta_laser_quat.x();
            auto y = delta_laser_quat.y();
            auto z = delta_laser_quat.z();

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            float yaw = std::atan2(siny_cosp, cosy_cosp);

            return Eigen::Affine2f(Eigen::Translation2f(delta_laser_trans.x(), delta_laser_trans.y()) * Eigen::Rotation2Df(yaw));
        }
    }

    void ScanPreprocess::setLaserTF(const double x, const double y, const double yaw)
    {
        laser_quat_ = Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        laser_trans_ = Eigen::Vector3f(x, y, 0);
    }

    void ScanPreprocess::setLaserAngle(const float angle_max, const float angle_min)
    {
        angle_max_ = angle_max;
        angle_min_ = angle_min;
    }

} // namespace last