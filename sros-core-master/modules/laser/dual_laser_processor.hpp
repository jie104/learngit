//
// Created by lfc on 19-8-27.
//

#ifndef SROS_DUAL_LASER_PROCESSOR_HPP
#define SROS_DUAL_LASER_PROCESSOR_HPP

#include <memory>
#include <Eigen/Dense>
#include <glog/logging.h>
#include "core/msg/laser_scan_msg.hpp"

namespace laser {
struct DualLaserPara {
    double first_center_x = 0.27f;
    double first_center_y = 0.17f;
    double first_center_yaw = 0.25f * M_PI;
    double first_angle_min = -0.75f * M_PI;
    double first_angle_max = 0.75f * M_PI;
    double first_origin_angle_min = -0.75f * M_PI;
    double first_origin_angle_max = 0.75f * M_PI;
    double second_center_x = -0.27f;
    double second_center_y = -0.17f;
    double second_center_yaw = -0.75f * M_PI;
    double second_angle_min = -0.75f * M_PI;
    double second_angle_max = 0.75f * M_PI;
    double second_origin_angle_min = -0.75f * M_PI;
    double second_origin_angle_max = 0.75f * M_PI;
    double obstacle_angle_cut_offset = 2.0 / 180 * M_PI;
    double angle_min = -M_PI;
    double angle_max = M_PI;
    double angle_increment = 2 * M_PI / 3600.0;
    double range_min = 0.05;
    double range_max = 30.0;
    const double laser_min_err = 0.05f;
};

typedef std::shared_ptr<DualLaserPara> DualLaserPara_Ptr;

struct DualPoint {
    Eigen::Vector2f second_coord_point;
    Eigen::Vector2f first_coord_point;
    float real_range;
    float real_angle;
    float inten;
    int index;
};
typedef std::shared_ptr<DualPoint> DualPoint_Ptr;
typedef std::vector<DualPoint_Ptr> DualPoints;

class DualLaserProcessor {
public:
    DualLaserProcessor(DualLaserPara_Ptr dual_para) : dual_para_(dual_para), delta_master_laser_tf_(Eigen::Affine2f::Identity()) {
        Eigen::Affine2f first_tf(Eigen::Translation2f(dual_para_->first_center_x, dual_para_->first_center_y) *
                                 Eigen::Rotation2Df(dual_para_->first_center_yaw));
        Eigen::Affine2f second_tf(Eigen::Translation2f(dual_para_->second_center_x, dual_para_->second_center_y) *
                                  Eigen::Rotation2Df(dual_para_->second_center_yaw));
        delta_laser_tf_ = first_tf.inverse() * second_tf;
        auto point = delta_laser_tf_ * Eigen::Vector2f(0.0, 0.0);
        LOG(INFO) << "point:" << point[0] << "," << point[1] << "," << atan2(point[1], point[0]);
        dual_points_size_ = computeDualPointSize(dual_para_->angle_increment, dual_para_->first_angle_max, dual_para_->first_angle_min);
        LOG(INFO) << "dual point size:" << dual_points_size_;

    }

    DualLaserPara_Ptr para(){ return dual_para_; }

    bool combineDualLidarPoints(sros::core::LaserScan_ptr first, sros::core::LaserScan_ptr second,
                                sros::core::LaserScan_ptr &dual_scan) {
        if (dual_para_->angle_increment != first->angle_increment) {
            dual_para_->angle_increment = first->angle_increment;
        }
        dual_para_->first_angle_min = dual_para_->first_origin_angle_min + dual_para_->obstacle_angle_cut_offset;
        dual_para_->first_angle_max = dual_para_->first_origin_angle_max - dual_para_->obstacle_angle_cut_offset;
        dual_para_->second_angle_min = dual_para_->second_origin_angle_min + dual_para_->obstacle_angle_cut_offset;
        dual_para_->second_angle_max = dual_para_->second_origin_angle_max - dual_para_->obstacle_angle_cut_offset;

        dual_points_size_ = computeDualPointSize(dual_para_->angle_increment, dual_para_->first_angle_max, dual_para_->first_angle_min);
        if (dual_points_size_ == 0) {
          return false;
        }

        auto angle = second->angle_min;
        auto angle_incre = second->angle_increment;
        auto half_incre = angle_incre / 2.0f;
        auto range_max = second->range_max;
        auto range_min = second->range_min;
        if (range_max > dual_para_->range_max) {
            range_max = dual_para_->range_max;
        }
        if (range_min < dual_para_->range_min) {
            range_min = dual_para_->range_min;
        }
        auto &ranges = second->ranges;
        auto &intens = second->intensities;
        auto &undistorted_ranges = second->undistorted_ranges;
        auto &undistorted_intens = second->undistorted_intensities;
        std::vector<DualPoints> dual_points_pair;
        std::vector<DualPoint_Ptr> dual_scan_points;
        std::vector<float> dual_pair_min_range(dual_points_size_, 10000.0f);
        dual_points_pair.resize(dual_points_size_);
        for(auto& pair:dual_points_pair){
            pair.reserve(10);
        }

        std::vector<DualPoints> undistorted_dual_points_pair;
        std::vector<DualPoint_Ptr> undistorted_dual_scan_points;
        std::vector<float> undistorted_dual_pair_min_range(dual_points_size_, 10000.0f);
        undistorted_dual_points_pair.resize(dual_points_size_);
        for(auto& pair:undistorted_dual_points_pair){
            pair.reserve(10);
        }

        if (ranges.empty()) {
            LOG(INFO) << "range empty!";
            return false;
        }
        auto last_range = ranges[0];
        auto last_inten = intens[0];
        auto last_undistorted_range = undistorted_ranges[0];
        auto last_undistorted_inten = undistorted_intens[0];
        auto cos_angle_incre = cosf(angle_incre);

        for (size_t i = 0; i < ranges.size(); ++i) {
            auto &range = ranges[i];
            auto & undistorted_range = undistorted_ranges[i];
            if (angle > dual_para_->second_angle_min && angle < dual_para_->second_angle_max) {
                if (range < range_max && range > range_min && last_range < range_max && last_range > range_min) {
                    DualPoint_Ptr dual_point;
                    buildDualPoint(angle, range, intens[i], dual_point, false);
                    addToDualPoints(dual_point->index, dual_point, dual_points_pair, dual_pair_min_range);
                    float pre_half_range = 2.0f * last_range * range * cos_angle_incre / (last_range + range);
                    auto pre_half_inten = (intens[i] + last_inten) * 0.5f;
                    auto pre_half_angle = angle - half_incre;
                    DualPoint_Ptr dual_half_point;
                    buildDualPoint(pre_half_angle, pre_half_range, pre_half_inten, dual_half_point, false);
                    addToDualPoints(dual_half_point->index, dual_half_point, dual_points_pair, dual_pair_min_range);
                }
                if (undistorted_range < range_max && undistorted_range > range_min && 
                                last_undistorted_range < range_max && last_undistorted_range > range_min)
                {
                    DualPoint_Ptr dual_point;
                    buildDualPoint(angle, undistorted_range, undistorted_intens[i], dual_point,true);
                    addToDualPoints(dual_point->index, dual_point, undistorted_dual_points_pair, undistorted_dual_pair_min_range);
                    float pre_half_range = 2.0f * last_undistorted_range * undistorted_range * cos_angle_incre /
                                           (last_undistorted_range + undistorted_range);
                    auto pre_half_inten = (undistorted_intens[i] + last_undistorted_inten) * 0.5f;
                    auto pre_half_angle = angle - half_incre;
                    DualPoint_Ptr dual_half_point;
                    buildDualPoint(pre_half_angle, pre_half_range, pre_half_inten, dual_half_point,true);
                    addToDualPoints(dual_half_point->index, dual_half_point, undistorted_dual_points_pair, undistorted_dual_pair_min_range);
                }
            }
            last_range = range;
            last_undistorted_range = undistorted_range;
            last_inten = intens[i];
            last_undistorted_inten = undistorted_intens[i];
            angle += angle_incre;
            // index++;
        }
        convertDualPointsToPoint(dual_pair_min_range, dual_points_pair, dual_scan_points);
        convertDualPointsToPoint(undistorted_dual_pair_min_range, undistorted_dual_points_pair, undistorted_dual_scan_points);
        convertPointToScan(first,second, dual_scan_points, undistorted_dual_scan_points, dual_scan);
        auto msg = dual_scan;
        int range_size = floorf((msg->angle_max - msg->angle_min) / msg->angle_increment + 0.5 + 1);
        if (range_size > msg->ranges.size()) {
            msg->angle_max -= msg->angle_increment;
        }
        return true;
    }


    void convertDualPointsToPoint(const std::vector<float> &dual_pair_min_range,
                                  const std::vector<DualPoints> &dual_points_pair, std::vector<DualPoint_Ptr> &points) {
        points.resize(dual_points_size_);
        int last_j = 0;
        DualPoint_Ptr last_point;
        for (int j = 0; j < dual_points_size_; ++j) {
            float point_size = 0;
            Eigen::Vector2f sum_point = Eigen::Vector2f::Zero();
            float sum_inten = 0;
            for (auto &point:dual_points_pair[j]) {
                if ((point->real_range - dual_pair_min_range[j]) < dual_para_->laser_min_err) {
                    sum_point += point->second_coord_point;
                    sum_inten += point->inten;
                    point_size += 1.0;
                }
            }
            if (point_size > 0.1f) {
                sum_point /= point_size;
                DualPoint_Ptr mean_point(new DualPoint);
                mean_point->second_coord_point = sum_point;
                mean_point->real_angle = atan2f(sum_point[1], sum_point[0]);
                mean_point->real_range = sum_point.norm();
                mean_point->inten = sum_inten / point_size;
                points[j] = mean_point;
                last_j = j;
                last_point = mean_point;
            } else {
                if (last_point) {
                    if ((j - last_j) == 1) {
                        points[j] = last_point;
                    }
                }
            }
        }
    }

    void convertPointToScan(sros::core::LaserScan_ptr &first,sros::core::LaserScan_ptr &second, std::vector<DualPoint_Ptr> &points,
                            std::vector<DualPoint_Ptr> &undistorted_points, sros::core::LaserScan_ptr &dual_scan)
    {
        if (!dual_scan) {
            dual_scan.reset(new sros::core::LaserScanMsg);
        }
        dual_scan->angle_min = dual_para_->angle_min;
        dual_scan->angle_max = dual_para_->angle_max;
        dual_scan->time_ = first->time_;
        dual_scan->first_angle_min = dual_para_->first_angle_min;
        dual_scan->first_angle_max = dual_para_->first_angle_max;
        dual_scan->second_angle_min = dual_para_->first_angle_max;
        dual_scan->second_angle_max = dual_scan->angle_max;
        dual_scan->sensor_name = first->sensor_name;
        dual_scan->second_sensor_name = second->sensor_name;
        dual_scan->angle_increment = first->angle_increment;
        dual_scan->range_min = first->range_min;
        dual_scan->range_max = first->range_max;
        auto &first_ranges = first->ranges;
        auto &first_undistorted_ranges = first->undistorted_ranges;
        auto &first_intens = first->intensities;
        auto &first_undistorted_intens = first->undistorted_intensities;
        auto angle_incre = first->angle_increment;
        auto angle = dual_scan->angle_min;
        auto &dual_ranges = dual_scan->ranges;
        auto &dual_intens = dual_scan->intensities;
        auto &dual_undistorted_ranges = dual_scan->undistorted_ranges;
        auto &dual_undistorted_intens = dual_scan->undistorted_intensities;
        int zero_count = 0;
        int undistort_zero_count = 0;

        while (angle < dual_scan->angle_max) {
            if (angle >= dual_para_->first_angle_min && angle < dual_para_->first_angle_max) {
                int index = (int) round((angle - first->angle_min) / angle_incre);
                dual_ranges.push_back(first_ranges[index]);
                dual_intens.push_back(first_intens[index]);
                dual_undistorted_ranges.push_back(first_undistorted_ranges[index]);
                dual_undistorted_intens.push_back(first_undistorted_intens[index]);
            } else {
                auto delta_angle = angle - dual_para_->first_angle_max;
                normalizeAngle(delta_angle);
                if (delta_angle < 0) {
                    delta_angle += M_PI;
                }
                int index = (int) round((delta_angle) / angle_incre);
                float curr_range = 0.0;
                float curr_undistorted_range = 0.0;
                float curr_inten = 0.0;
                float curr_undistorted_inten = 0.0;
                if (index >= 0 && index < dual_points_size_) {
                    auto &curr_point = points[index];
                    auto &curr_undistorted_point = undistorted_points[index];
                    DualPoint_Ptr other_point;
                    if (index == 0 || index == dual_points_size_ - 1) {
                        if (curr_point) {
                            curr_range = curr_point->real_range;
                            curr_inten = curr_point->inten;
                        }
                        if (curr_undistorted_point)
                        {
                            curr_undistorted_range = curr_undistorted_point->real_range;
                            curr_undistorted_inten = curr_undistorted_point->inten;
                        }
                    } else {
                        if (curr_point) {
                            curr_inten = curr_point->inten;
                            if (angle > curr_point->real_angle) {
                                auto &point_next = points[index + 1];
                                if (point_next) {
                                    other_point = point_next;
                                }
                            } else {
                                auto &point_pre = points[index - 1];
                                if (point_pre) {
                                    other_point = point_pre;
                                }
                            }
                            if (other_point && (other_point != curr_point)) {
                                curr_range = other_point->real_range * curr_point->real_range *
                                             sinf(other_point->real_angle - curr_point->real_angle) /
                                             (other_point->real_range * sinf(other_point->real_angle - angle) +
                                              curr_point->real_range * sinf(angle - curr_point->real_angle));

                            } else {
                                curr_range = curr_point->real_range;
                            }
                        }else{
                            zero_count++;
                        }

                        if (curr_undistorted_point)
                        {
                            DualPoint_Ptr other_undistorted_point;
                            curr_undistorted_inten = curr_undistorted_point->inten;
                            if (angle > curr_undistorted_point->real_angle)
                            {
                                auto &point_next = undistorted_points[index + 1];
                                if (point_next)
                                {
                                    other_undistorted_point = point_next;
                                }
                            }
                            else
                            {
                                auto &point_pre = undistorted_points[index - 1];
                                if (point_pre)
                                {
                                    other_undistorted_point = point_pre;
                                }
                            }
                            if (other_undistorted_point && (other_undistorted_point != curr_undistorted_point))
                            {
                                curr_undistorted_range = other_undistorted_point->real_range * curr_undistorted_point->real_range *
                                             sinf(other_undistorted_point->real_angle - curr_undistorted_point->real_angle) /
                                             (other_undistorted_point->real_range * sinf(other_undistorted_point->real_angle - angle) +
                                              curr_undistorted_point->real_range * sinf(angle - curr_undistorted_point->real_angle));
                            }
                            else
                            {
                                curr_undistorted_range = curr_undistorted_point->real_range;
                            }
                        }
                        else
                        {
                            undistort_zero_count++;
                        }
                    }
                } else {
                    LOG(INFO) << "index is err!" << index << dual_points_size_;
                }

                dual_ranges.push_back(curr_range);
                dual_intens.push_back(curr_inten);
                dual_undistorted_ranges.push_back(curr_undistorted_range);
                dual_undistorted_intens.push_back(curr_undistorted_inten);
            }
            angle += angle_incre;
        }
    }

    void setDeltaFirstLaser(const Eigen::Affine2f &delta_tf)
    {
        delta_master_laser_tf_ = delta_tf;
    }


    template<class T>
    static void normalizeAngle(T &angle) {
        angle = fmod(angle, 2.0 * M_PI);
        if (angle >= M_PI) {
            angle -= 2.0f * M_PI;
        } else if (angle < -M_PI) {
            angle += 2.0f * M_PI;
        }
    }

private:
    int computeDualPointSize(const float& angle_incre,const float& angle_max,const float& angle_min){
        double delta_angle = angle_max - angle_min;
        if (delta_angle <= 0) {
            LOG(INFO) << "delta angle is wrong! cannot combine dual laser processor!";
            return 0;
        }
        delta_angle = 2 * M_PI - delta_angle;
        auto dual_points_size = int(delta_angle / angle_incre + 0.5f) + 1;
//        LOG(INFO) << "dual points size:" << dual_points_size;
        return dual_points_size;
    }

    void buildDualPoint(const float &angle, const float &range, const float inten, DualPoint_Ptr &dual_point,const bool is_undistorted) {
        dual_point.reset(new DualPoint);
        auto &first_point = dual_point->first_coord_point;
        first_point = Eigen::Vector2f(range * cos(angle), range * sin(angle));
        if (is_undistorted)
            dual_point->second_coord_point = delta_master_laser_tf_ * delta_laser_tf_ * first_point;
        else
            dual_point->second_coord_point = delta_laser_tf_ * first_point;
        dual_point->real_angle = atan2f(dual_point->second_coord_point[1], dual_point->second_coord_point[0]);
        dual_point->real_range = dual_point->second_coord_point.norm();
        double delta_angle = dual_point->real_angle - dual_para_->first_angle_max;
        normalizeAngle(delta_angle);
        if (delta_angle < 0) {
            delta_angle += M_PI;
        }
        dual_point->index = (int) round((delta_angle) / dual_para_->angle_increment);
        dual_point->inten = inten;
    }

    void addToDualPoints(int index, const DualPoint_Ptr &dual_point, std::vector<DualPoints> &dual_points_pair,
                         std::vector<float> &dual_pair_min_range) {
        if (index >= 0 && index < dual_points_size_) {
            dual_points_pair[index].push_back(dual_point);
            auto &min_range = dual_pair_min_range[index];
            min_range = min_range < dual_point->real_range ? min_range : dual_point->real_range;
        }
    }


    DualLaserPara_Ptr dual_para_;
    Eigen::Affine2f delta_laser_tf_;
    Eigen::Affine2f delta_master_laser_tf_;
    int dual_points_size_;


};


}


#endif //SROS_DUAL_LASER_PROCESSOR_HPP
