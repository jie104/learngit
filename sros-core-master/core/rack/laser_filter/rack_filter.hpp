

//
// Created by lfc on 19-1-3.
//

#ifndef PROJECT_RACK_FILTER_HPP
#define PROJECT_RACK_FILTER_HPP

#include <memory>
#include <Eigen/Dense>
//#include <sensor_msgs/LaserScan.h>
#include <glog/logging.h>
#include "rack_para.hpp"

namespace rack {
//struct RackPara{
//    double rack_width = 0.8;
//    double rack_length = 1.35;
//    double rack_leg_radius = 0.05;
//};


struct RadarPointInfo {
    Eigen::Vector2d point;
};

struct ClusterPoint {
    int min_index;
    float min_index_range;
    int max_index;
    float max_index_range;
    std::vector<RadarPointInfo> infos;
};

typedef std::shared_ptr<ClusterPoint> ClusterPoint_Ptr;

struct Rectangle {
    double min_y = 0.6 / 2;
    double max_y = -0.6 / 2;
    double min_x = -1.06 / 2.0;
    double max_x = 1.06 / 2.0;
};

typedef std::shared_ptr<Rectangle> Rectangle_Ptr;

class RectangleInfo {
public:
    void updateInfo(Rectangle_Ptr rectangle, double direction) {
        rectangle_ = rectangle;
        direction_offset_ = direction;
        cos_theta = cos(direction_offset_);
        sin_theta = sin(direction_offset_);
    }

    void computeTFInfo(double direction) {
        cos_theta = cos(direction_offset_ + direction);
        sin_theta = sin(direction_offset_ + direction);
    }

    bool inRectangle(const Eigen::Vector2d &point) {
        double tmp_x = point[0] * cos_theta + point[1] * sin_theta;
        double tmp_y = point[1] * cos_theta - point[0] * sin_theta;
        if (tmp_x >= rectangle_->min_x && tmp_x <= rectangle_->max_x) {
            if (tmp_y >= rectangle_->min_y && tmp_y <= rectangle_->max_y) {
                return true;
            }
        }
        return false;
    }

#ifdef ROS_DEBUG_API
    public:
#else
private:
#endif

    Rectangle_Ptr rectangle_;
    double direction_offset_ = 0.0;

    double cos_theta;
    double sin_theta;
};

//typedef sensor_msgs::LaserScanPtr ScanType;

template<class ScanType>
class RackFilter {
public:
    RackFilter(InstallPara &install_para) : install_para_(install_para) {
        rack_rectangle_.reset(new Rectangle);
        rack_leg_angle_offset = install_para_.backlash_angle;
        int num_times = round(install_para_.backlash_angle / rectangle_angle_step);
        for (int i = -num_times; i <= num_times; ++i) {
            rectangles_.emplace_back();
            rectangles_.back().updateInfo(rack_rectangle_, double(i) * rectangle_angle_step);
        }
        scan_to_center_tf_ = Eigen::Translation2d(install_para.laser_coord_x, install_para.laser_coord_y) *
                             Eigen::Rotation2Dd(install_para.laser_coord_yaw);
    }

    void computeRackInfo(RackInfo_Ptr &rack_info) {
        auto &rack = rack_info->rack_para;
        double max_x = (rack.rack_length / 2.0 + rack.rack_leg_radius + laser_std_err);
        double min_x = -max_x;
        double max_y = (rack.rack_width / 2.0 + rack.rack_leg_radius + laser_std_err);
        double min_y = -max_y;
        rack_rectangle_->max_x = max_x;
        rack_rectangle_->max_y = max_y;
        rack_rectangle_->min_x = min_x;
        rack_rectangle_->min_y = min_y;
        LOG(INFO) << "clear rack!";
        check_rack_circle_.clear();
    }
    void computeRackInfo(RackPara &rack) {
        double max_x = (rack.rack_length / 2.0 + rack.rack_leg_radius + laser_std_err);
        double min_x = -max_x;
        double max_y = (rack.rack_width / 2.0 + rack.rack_leg_radius + laser_std_err);
        double min_y = -max_y;
        rack_rectangle_->max_x = max_x;
        rack_rectangle_->max_y = max_y;
        rack_rectangle_->min_x = min_x;
        rack_rectangle_->min_y = min_y;
        check_rack_circle_.clear();
    }

    void filterRack(ScanType &scan, double curr_direction = 0.0) {
        std::vector<ClusterPoint_Ptr> rack_clusters;
        filterRack(scan, rack_clusters, curr_direction);
    }

    void filterRack(ScanType &scan, std::vector<ClusterPoint_Ptr> rack_clusters, double curr_direction = 0.0) {
        if (check_rack_circle_.empty()) {
            computeCircleInfo(scan);
        }
        std::vector<bool> points_state(scan->ranges.size(), true);
        filterOutBoarderPoint(scan, install_para_);
        filterLowIntenPoint(scan);
        std::vector<int> trailing_indexes;
        filterScanTrailingPoint(scan, points_state, 2, 0.02);
        std::vector<ClusterPoint_Ptr> clusters;
        extractClusters(scan, points_state, clusters);

        findRackCluster(clusters, rack_clusters, curr_direction);
        dilateClusters(scan, rack_clusters);
        filterRackClusters(scan, rack_clusters);
        int point_size = points_state.size();
        for (int i = 0; i < point_size; ++i) {
            if (!points_state[i]) {
                scan->ranges[i] = 0;
            }
        }
    }

    void extractClusters(ScanType &scan, std::vector<bool> &points_state, std::vector<ClusterPoint_Ptr> &clusters) {
        auto &ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        int index = 0;
        clusters.clear();
        clusters.emplace_back(new (ClusterPoint));
        for (auto &range:ranges) {
            if (range < range_min || range > check_rack_circle_[index] || !points_state[index]) {
                if (clusters.back()->infos.empty()) {

                } else if (clusters.back()->infos.size() == 1) {//孤立点
                    ranges[clusters.back()->min_index] = 0;
                    clusters.back()->infos.clear();
                } else {
                    clusters.emplace_back(new (ClusterPoint));
                }
            } else {
                if (clusters.back()->infos.empty()) {
                    clusters.back()->min_index = index;
                    clusters.back()->min_index_range = range;
                }
                clusters.back()->infos.emplace_back();
                clusters.back()->max_index = index;
                clusters.back()->max_index_range = range;
                auto &info = clusters.back()->infos.back();
                info.point = Eigen::Vector2d(range * cos(curr_angle), range * sin(curr_angle));
            }
            index++;
            curr_angle += angle_incre;
        }
        if (clusters.back()->infos.empty()) {
            clusters.resize(clusters.size() - 1);
        }
    }

    void dilateClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &clusters) {
        double increment = scan->angle_increment;
        auto &ranges = scan->ranges;
        int max_size = scan->ranges.size() - 1;
        int delta_trailing_size = round(dilate_cluster_angle_offset / scan->angle_increment);
        for (auto &cluster:clusters) {
            auto curr_trailing_size =
                    (cluster->min_index - delta_trailing_size) > 0 ? delta_trailing_size : cluster->min_index;
            dilateClusterBoarder(ranges, increment, cluster->min_index, curr_trailing_size, -1);
            curr_trailing_size = (cluster->max_index + delta_trailing_size) > max_size ? (max_size - cluster->max_index)
                                                                                     : delta_trailing_size;
            dilateClusterBoarder(ranges, increment, cluster->max_index, curr_trailing_size, 1);
        }
    }

    void dilateClusterBoarder(const std::vector<float> &ranges, const double increment, int &cluster_index,
                              int dilate_size, int direction = 1) {
        const int beg_index = cluster_index;
        auto ref_range = ranges[beg_index];
        for (int i = 1; i < dilate_size; ++i) {
            auto first_edge_index = beg_index + i * direction;
            auto range = ranges[first_edge_index];
            auto delta_dist = range * range + ref_range * ref_range - 2.0 * range * ref_range * cos(i * increment);
            if (delta_dist <= dilate_cluster_dist_thresh * dilate_cluster_dist_thresh) {
                cluster_index = first_edge_index;
            }
        }
    }

    void filterRackClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &rack_clusters) {
        int max_size = scan->ranges.size() - 1;
        int delta_trailing_size = round(rack_leg_angle_offset / scan->angle_increment);
        auto &ranges = scan->ranges;
        for (auto &cluster:rack_clusters) {
            auto curr_trailing_size =
                    cluster->min_index - delta_trailing_size > 0 ? delta_trailing_size : cluster->min_index;
            filterTrailingPointByRange(ranges, cluster->min_index, curr_trailing_size, -1, cluster->min_index_range);
            curr_trailing_size = cluster->max_index + delta_trailing_size > max_size ? max_size - cluster->max_index
                                                                                     : delta_trailing_size;
            filterTrailingPointByRange(ranges, cluster->max_index, curr_trailing_size, 1, cluster->max_index_range);
            auto &max_index = cluster->max_index;
            for (int i = cluster->min_index; i <= max_index; ++i) {
                scan->ranges[i] = 0;
            }
        }
    }

    void findRackCluster(std::vector<ClusterPoint_Ptr> &clusters, std::vector<ClusterPoint_Ptr> &rack_clusters,
                         double direction) {
        for (auto &rectangle:rectangles_) {
            rectangle.computeTFInfo(direction);
        }
        rack_clusters.clear();
        for (auto &cluster:clusters) {
            int cluster_in_rack_count = 0;
            for (auto &point:cluster->infos) {
                auto center_point = scan_to_center_tf_ * point.point;
                for (auto &rectangle:rectangles_) {
                    if (rectangle.inRectangle(center_point)) {
                        cluster_in_rack_count++;//如果当前点在Center内，则认为该点合理。
                        break;
                    }
                }
            }
            int cluster_size = cluster->infos.size();
            if (cluster_size > 0) {
                double percent = (double) cluster_in_rack_count / (double) cluster_size;
                if (percent > cluster_in_rectangle_thresh) {
                    rack_clusters.push_back(cluster);
                }
            }
        }

    }

    void filterOutBoarderPoint(ScanType &scan, InstallPara &install_para) {
        auto &ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        for (auto &range:ranges) {
            if (range < range_min || range > range_max) {
                range = 0;
            } else {
                if (curr_angle < install_para.laser_angle_min || curr_angle > install_para.laser_angle_max) {
                    range = 0;
                }
            }
            curr_angle += angle_incre;
        }
    }


    void filterScanTrailingPoint(ScanType &scan, std::vector<bool> &points_state, int step = 2,
                                 double min_thresh = 0.02) {//注意，这里如果拖尾部分有一个点被滤掉了，那其他点则无法识别出为拖尾点了，因为，这里有距离判断
        auto &ranges = scan->ranges;

        double cos_increment = cos(scan->angle_increment * (double) step * 2.0);
        double theta_thresh = sin((double) scan->angle_increment * (double) step * 2.0) / sin(0.17);//临界值,用于识别断点

        int scan_size = ranges.size() - step;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 0 || ranges[i] > check_rack_circle_[i]) {
                continue;
            }

            double dist_1 = std::sqrt(ranges[i + step] * ranges[i + step] + ranges[i] * ranges[i] -
                                      2 * ranges[i + step] * ranges[i] * cos_increment);

            double range_thresh_1 = ranges[i + step] * theta_thresh + min_thresh;
            if (dist_1 > range_thresh_1) {
                int remove_gap = step;
                for (int j = -remove_gap; j <= remove_gap; ++j) {
                    points_state[i + j] = false;
                }
            }
        }
    }


    void filterTrailingPointByIndex(ScanType &scan, int min_index, int delta_index) {

        auto &ranges = scan->ranges;
        std::vector<int> indexes;
        double cos_increment = cos(scan->angle_increment);
        double theta_thresh = sin((double) scan->angle_increment) / sin(0.17);//临界值,用于识别断点
        for (int i = min_index + 1; i < delta_index; ++i) {
            double dist_1 = std::sqrt(ranges[i + 1] * ranges[i + 1] + ranges[i] * ranges[i] -
                                      2 * ranges[i + 1] * ranges[i] * cos_increment);

            double range_thresh_1 = ranges[i] * theta_thresh + min_trailing_thresh;

            double dist_2 = std::sqrt(ranges[i] * ranges[i] + ranges[i - 1] * ranges[i - 1] -
                                      2 * ranges[i] * ranges[i - 1] * cos_increment);

            double range_thresh_2 = ranges[i] * theta_thresh + min_trailing_thresh;
            if (dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
                indexes.push_back(i);
            }
        }
        for (auto &index:indexes) {
            ranges[index] = 100;
        }
    }

    void filterTrailingPointByRange(std::vector<float> &ranges, int min_index, int delta_index, int step,
                                    double ref_range) {
        const double MAX_LEG_RANGE_THERSH = 1.0; // 单位m
        int max_index = min_index + delta_index;
        for (int j = 1; j <= delta_index; ++j) {
            const int &index = min_index + j * step;
            if (inRange(ranges[index], ref_range, MAX_LEG_RANGE_THERSH)) {
                ranges[index] = 100;
            }
        }
    }

private:
    void filterLowIntenPoint(ScanType &scan) {
        int scan_size = scan->ranges.size();
        auto &ranges = scan->ranges;
        auto &intens = scan->intensities;

        for (int i = 0; i < scan_size; ++i) {
            if (ranges[i] < check_rack_circle_[i]) {//判断当前range是否在要进行判断的范围之内
                if (intens[i] < low_inten_thresh) {
                    scan->ranges[i] = 0;
                }
            }
        }
    }

    void computeCircleInfo(ScanType &scan) {
        check_rack_circle_.clear();
        check_rack_circle_.resize(scan->ranges.size());
        LOG(INFO) << "begin to compute the rack info!";
        double min_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = min_angle;
        double radius_thresh = (sqrt(rack_rectangle_->max_x * rack_rectangle_->max_x +
                                     rack_rectangle_->max_y * rack_rectangle_->max_y) + check_dist_offset);//计算允许过滤的最远距离


        Eigen::Affine2f laser_tf(
                Eigen::Translation2f(install_para_.laser_coord_x, install_para_.laser_coord_y) *
                Eigen::Rotation2Df(install_para_.laser_coord_yaw));
        Eigen::Vector2f center_pose = laser_tf.inverse() * Eigen::Vector2f(0, 0);
        double a_0 = center_pose[0];
        double b_0 = center_pose[1];

        double r_0_2 = a_0 * a_0 + b_0 * b_0;
        double r_0 = sqrt(r_0_2);
        double phi = atan2(b_0, a_0);
        LOG(INFO) << "radius:" << radius_thresh << "," << a_0 << "," << b_0 << "," << phi;
        for (auto &range:check_rack_circle_) {//将所有角度的range均计算出过滤的最远距离
            //利用极坐标直线公式:r*r - 2*r*r0cos(theta-phi)+r0*r0-a*a,

            double cos_delta_theta = cos(angle - phi);
            double b = -2 * r_0 * cos_delta_theta;
            double k = r_0_2 - radius_thresh * radius_thresh;
            double delta_2 = b * b - 4 * k;
            if (delta_2 < 0) {
                LOG(WARNING) << "the delta_2 is err!" << delta_2;
                range = 0;
            } else {
                double range_1 = (-b + sqrt(delta_2)) / 2.0;
                double range_2 = (-b - sqrt(delta_2)) / 2.0;

                range = std::max(range_1, range_2);
                if (range < 0) {
                    LOG(WARNING) << "the range is err!" << range;
                    range = 0;
                }
            }
            angle += angle_incre;
        }
    }


    inline bool inRange(double dist, double dist_thresh, double offset) {
        return dist < (dist_thresh + offset);
    }

#ifdef ROS_DEBUG_API
    public:
#else
private:
#endif
    const double low_inten_thresh = 150;
    const double laser_std_err = 0.03;
    const double check_dist_offset = 1.0;
    const double rectangle_angle_step = 2.0 * M_PI / 180.0;
    const double cluster_in_rectangle_thresh = 0.8;
    const double min_trailing_thresh = 0.02;
    double rack_leg_angle_offset = 2.0 * M_PI / 180.0;
    const double dilate_cluster_angle_offset = 1.0 * M_PI / 180.0;
    const double dilate_cluster_dist_thresh = 0.05;
    std::vector<double> check_rack_circle_;

    std::vector<RectangleInfo> rectangles_;
    InstallPara install_para_;
    Rectangle_Ptr rack_rectangle_;
    Eigen::Affine2d scan_to_center_tf_;
};


}


#endif //PROJECT_RACK_FILTER_HPP
