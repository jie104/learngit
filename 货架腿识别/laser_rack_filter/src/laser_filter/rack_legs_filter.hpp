//
// Created by lfc on 18-8-16.
//

#ifndef SROS_RACK_LEGS_FILTER_HPP
#define SROS_RACK_LEGS_FILTER_HPP

#include <Eigen/Dense>
#include <vector>
#include <glog/logging.h>
#include "motion_prediction.hpp"
//#include "core/msg/laser_scan_msg.hpp"


namespace laser {

struct RackPara {
    RackPara() = default;

    // 初始搜索角度范围阈值
    double rack_leg_search_thresh = 10.0f * M_PI / 180.0f;

    // 拖尾点角度范围阈值
    double tailing_noise_angle_thresh = 1.0f * M_PI / 180.0f;

    // 货架所有leg中心点组成的矩形width，单位m
    double rack_leg_center_width = 0.57f;

    // 货架所有leg中心点组成的矩形length，单位m
    double rack_leg_center_length = 1.02f;

    // 货架leg的直径，单位m
    double rack_leg_diameter = 0.1f;

    bool use_region_filter = true;

    double rack_radius_offset = 0.05f;//货架上最远位置调节量,防止货架顶偏
};

template<typename ScanType>
class RackLegsFilter {
public:
    struct LegPair{
        Eigen::Vector3d mid_pose;
        Eigen::Vector2d first_leg;
        Eigen::Vector2d second_leg;
        double angle;
    };

    struct ComputeInfo {
        Eigen::Vector3d rack_center_pose;
        std::vector<Eigen::Vector2d> legs;
        std::vector<std::vector<double>> regions;
    };

    struct RangeInfo {
        int index;
        double range;
        double angle;
    };

    struct ClustterInfo {
        void computeClusterInfo() {
            double sum_x = 0;
            double sum_y = 0;
            min_index = 100000000;
            max_index = 0;
            is_leg = false;
            double size = points.size();
            if (size == 0) {
                LOG(WARNING) << "zero point!";
                min_index = 0;
                max_index = 0;
                center_point = Eigen::Vector2d::Zero();
                return;
            }
            for (auto &point:points) {
                min_index = point.index < min_index ? point.index : min_index;
                max_index = point.index > max_index ? point.index : max_index;
                sum_x += cos(point.angle) * point.range;
                sum_y += sin(point.angle) * point.range;
            }
            center_point[0] = sum_x / size;
            center_point[1] = sum_y / size;
        }

        bool is_leg;
        std::vector<RangeInfo> points;
        int min_index;
        int max_index;
        Eigen::Vector2d center_point;
    };

    RackLegsFilter(const Eigen::Vector3d laser_pose = Eigen::Vector3d(0.345f, 0.0f, 0.0f),
                   const RackPara para = RackPara()) : rack_para_(para), laser_pose_(laser_pose) {
        recomputePara();
    }

    void setLaserPose(const Eigen::Vector3d &laser_pose) {
        laser_pose_ = laser_pose;
        recomputePara();
    }

    void setRackPara(const RackPara &rack_para) {
        // 通过该值设置rack的长和宽;如果长宽在运动过程中改变,设置该参数即可.
        rack_para_ = rack_para;
        recomputePara();
    }

    void setLaserAngleRange(double angle_min, double angle_max) {
        laser_angle_min_ = angle_min;
        laser_angle_max_ = angle_max;
        recomputePara();
    }

    void recomputePara() {
        Eigen::Affine2f laser_tf(
                Eigen::Translation2f(laser_pose_[0], laser_pose_[1]) * Eigen::Rotation2Df(laser_pose_[2]));
        // 这里有假设，货架中心位于车体中心
        // 沿着正方向的尺寸为长,垂直正方向的尺寸为宽
        double width_2 = rack_para_.rack_leg_center_width / 2.0f;
        double length_2 = rack_para_.rack_leg_center_length / 2.0f;

        Eigen::Vector2f leg_1(length_2, width_2);//第一个腿
        Eigen::Vector2f leg_2(-length_2, width_2);//第二个腿
        Eigen::Vector2f leg_3(-length_2, -width_2);//第三个腿
        Eigen::Vector2f leg_4(length_2, -width_2);//第四个腿

        // 注意这里,laser位姿为运动中心为坐标零点的位姿,因此需要将货架腿变换到laser中心
        std::vector<Eigen::Vector2f> laser_legs;
        laser_legs.push_back(laser_tf.inverse() * leg_1);
        laser_legs.push_back(laser_tf.inverse() * leg_2);
        laser_legs.push_back(laser_tf.inverse() * leg_3);
        laser_legs.push_back(laser_tf.inverse() * leg_4);

        laser_legs_ = laser_legs;//记录leg位置信息.
        range_region.clear();
        rack_circle.clear();

        leg_infos_.clear();
        for (const auto &laser_leg : laser_legs) {
            // 计算每个腿的位置

            LegInfo leg_info;
            leg_info.laser_angle = normalizeAngle(atan2(laser_leg[1], laser_leg[0]));

            // 其中的laser_angle_min是小车的视野,存在视野内观测到多个腿的情况
            if (leg_info.laser_angle > laser_angle_min_ && leg_info.laser_angle < laser_angle_max_) {
                leg_info.laser_dist = laser_leg.norm();//计算货架腿距离雷达中心的中心位置
                leg_infos_.push_back(leg_info);//将满足要求的腿存储到腿容器内
            }
        }
    }

    void filterScan(ScanType &scan) {
        for (const auto &leg : leg_infos_) {
            // 将所有的腿依次滤去
            filterNoiseLeg(scan, leg);
        }
        if (rack_para_.use_region_filter) {
            filterScanByRegion(scan);//将区域内雷达全部滤去
        }
        //TODO:如果铁链存在,目前依然无法过滤掉.因为铁链大概率会产生拖尾问题.因此,后期还需要针对该问题具体解决.
    }

    void filterRack(ScanType &scan) {

        std::vector<RangeInfo> RackPoints;
        if (rack_circle.size() == 0) {
            computeCircleInfo(scan);
        }

        if (findRackPoints(scan, RackPoints)) {
            std::vector<ClustterInfo> clusters;
            splitPoints(RackPoints, clusters);
            if (clusters.size() < 1) {
                LOG(WARNING) << "the cluster size is wrong!" << clusters.size();
                motion_prediction.reset();
                return;
            }
            std::vector<ClustterInfo> legs_cluster;
            findRackLegs(clusters, legs_cluster);
            if (legs_cluster.size() == 0) {
                if (motion_prediction.isRotate(5.0 * M_PI / 180)) {
                    LOG(INFO) << "rotate!";
                    std::vector<RangeInfo> circle_points;
                    findAllCirclePoints(scan, circle_points);
                    std::vector<ClustterInfo> circle_clusters;
                    splitPoints(circle_points, circle_clusters);
                    for (auto &cluster:circle_clusters) {
                        filterNoiseCluster(scan, cluster);
                    }
                }
                return;
            }
            for (auto &cluster:legs_cluster) {
                filterNoiseCluster(scan, cluster);
            }
            std::vector<std::vector<Eigen::Vector2d> > multi_legs;
            if (rack_para_.use_region_filter) {
                creatLegPara(legs_cluster, multi_legs);
                compute_info.legs.clear();
                compute_info.regions.clear();
//                LOG(INFO) << "multi legs:" << multi_legs.size();
                for (auto &legs:multi_legs) {
                    if (legs.size()) {

                        for (auto &leg:legs) {
                            compute_info.legs.push_back(leg);
                        }
                        computeRegionInfo(scan, legs);
                        std::vector<RangeInfo> noise_points;
                        if(findRectanglePoints(scan, noise_points)){
                            std::vector<ClustterInfo> noise_clusters;
                            splitPoints(noise_points, noise_clusters);
                            for (auto &cluster:noise_clusters) {
                                filterNoiseCluster(scan, cluster);
                            }
                        }
//                        filterScanByRegion(scan);
                        compute_info.regions.push_back(range_region);
                    }
                }
            }
        }
    }

    ComputeInfo &getComputeInfo() {
        return compute_info;
    }

private:
    void filterNoiseCluster(ScanType &scan, ClustterInfo &cluster) {
        // 将搜索到的leg范围加上拖尾点范围，得到需要滤掉的准确index范围
        auto &ranges = scan->ranges;
        int range_max_index = ranges.size() - 1;
        int delta_tail_noise_index = (unsigned long) lround(
                2 * rack_para_.tailing_noise_angle_thresh / scan->angle_increment);

        int index_min, index_max;
        index_min = cluster.min_index - delta_tail_noise_index;
        index_min = (index_min < 0) ? 0 : index_min;

        index_max = cluster.max_index + delta_tail_noise_index;
        index_max = (index_max > range_max_index) ? range_max_index : index_max;

        assert(index_min <= index_max);

        // 如果某个点的距离大于leg实际距离+下面的阈值，那么不认为其是leg
        const double MAX_LEG_RANGE_THERSH = 1.0; // 单位m

        // 将搜索到的leg范围内的点range全部清零
        for (auto i = index_min; i < index_max; ++i) {
            if (inRange(ranges[i], rack_circle[i], MAX_LEG_RANGE_THERSH)) {
                scan->ranges[i] = 0; // 直接修改参数scan->ranges
            }
        }
        for (auto &point:cluster.points) {
            ranges[point.index] = 0;
        }
    }

    bool findRackPoints(ScanType &scan, std::vector<RangeInfo> &rack_points) {
        auto &ranges = scan->ranges;
        int range_size = ranges.size();
        double angle_min = scan->angle_min;
        double angle_incre = scan->angle_increment;
        for (int i = 0; i < range_size; ++i) {
            auto &range = ranges[i];
//            LOG(INFO) <<"range:"<<range<<","<<rack_circle[i];
            if (inMaxAndMinRange(range, rack_circle[i], rack_para_.rack_leg_diameter/2.0, rack_para_.rack_leg_diameter/2.0)) {
//                LOG(INFO) << "range:" << range << "," << rack_circle[i] << "," << rack_para_.rack_leg_diameter << "," <<
//                i << "," << angle_min + i * angle_incre;
                RangeInfo range_info;
                range_info.angle = angle_min + (double) i * angle_incre;
                range_info.range = range;
                range_info.index = i;
                rack_points.push_back(range_info);
            }
        }//将在货架腿环内所有点找到.
        return rack_points.size() > 0;
    }

    bool findAllCirclePoints(ScanType &scan, std::vector<RangeInfo> &rack_points) {
        auto &ranges = scan->ranges;
        int range_size = ranges.size();
        double angle_min = scan->angle_min;
        double angle_incre = scan->angle_increment;
        for (int i = 0; i < range_size; ++i) {
            auto &range = ranges[i];
//            LOG(INFO) <<"range:"<<range<<","<<rack_circle[i];
            if (inRange(range, rack_circle[i], rack_para_.rack_leg_diameter/2.0)) {
//                LOG(INFO) << "range:" << range << "," << rack_circle[i] << "," << rack_para_.rack_leg_diameter << "," <<
//                i << "," << angle_min + i * angle_incre;
                RangeInfo range_info;
                range_info.angle = angle_min + (double) i * angle_incre;
                range_info.range = range;
                range_info.index = i;
                rack_points.push_back(range_info);
            }
        }//将在货架腿环内所有点找到.
        return rack_points.size() > 0;
    }

    bool findRectanglePoints(ScanType &scan, std::vector<RangeInfo> &rectangle_points) {
        auto &ranges = scan->ranges;
        int range_size = ranges.size();
        double angle_min = scan->angle_min;
        double angle_incre = scan->angle_increment;
        for (int i = 0; i < range_size; ++i) {
            auto &range = ranges[i];
//            LOG(INFO) <<"range:"<<range<<","<<rack_circle[i];
            if (inRange(range, range_region[i], rack_para_.rack_radius_offset)) {
//                LOG(INFO) << "range:" << range << "," << rack_circle[i] << "," << rack_para_.rack_leg_diameter << "," <<
//                i << "," << angle_min + i * angle_incre;
                RangeInfo range_info;
                range_info.angle = angle_min + (double) i * angle_incre;
                range_info.range = range;
                range_info.index = i;
                rectangle_points.push_back(range_info);
            }
        }//将货架区域内所有点找到.
        return rectangle_points.size() > 0;
    }

    void splitPoints(const std::vector<RangeInfo> &rack_points, std::vector<ClustterInfo> &clusters) {
        if (rack_points.size() == 0) {
            LOG(WARNING) << "no point!";
            return;
        }
        clusters.emplace_back();
        double delta_angle_offset = 0.5 * M_PI / 180.0;//0.5°,5个点

        for (auto &point:rack_points) {
            auto &cluster_points = clusters.back().points;
            if (cluster_points.size() == 0) {
                cluster_points.push_back(point);
            } else if (nearWithCluster(cluster_points, point, delta_angle_offset)) {
                cluster_points.push_back(point);
            } else {
                clusters.emplace_back();
                clusters.back().points.push_back(point);
            }
        }
        for (auto &cluster:clusters) {
            cluster.computeClusterInfo();
        }
    }

    void findRackLegs(std::vector<ClustterInfo> &clusters, std::vector<ClustterInfo> &legs) {
        int cluster_size = clusters.size();
        double range_offset = rack_para_.rack_leg_diameter;
        double leg_width = rack_para_.rack_leg_center_width;
        double leg_length = rack_para_.rack_leg_center_length;

        for (int i = 0; i < cluster_size; ++i) {
            auto &fir_cluster = clusters[i];
            for (int j = i + 1; j < cluster_size; ++j) {
                auto &sec_cluster = clusters[j];
                double cluster_dist = (fir_cluster.center_point - sec_cluster.center_point).norm();
                if (inAbsRange(cluster_dist, leg_width, range_offset) ||
                    inAbsRange(cluster_dist, leg_length, range_offset)) {
                    fir_cluster.is_leg = true;
                    sec_cluster.is_leg = true;
                }
            }
        }
        for (auto &cluster:clusters) {
            if (cluster.is_leg) {
                legs.push_back(cluster);
            }
        }
    }

    bool findValidLegs(std::vector<ClustterInfo> &legs_cluster, std::vector<LegPair>& leg_pairs) {
        int legs_size = legs_cluster.size();
        Eigen::Affine2d laser_tf(
                Eigen::Translation2d(laser_pose_[0], laser_pose_[1]) * Eigen::Rotation2Dd(laser_pose_[2]));

        for (int i = 0; i < legs_size; ++i) {

            for (int j = i + 1; j < legs_size; ++j) {
                auto mid_point = (legs_cluster[i].center_point + legs_cluster[j].center_point) / 2.0;
                auto delta_point = (legs_cluster[j].center_point - legs_cluster[i].center_point);
                double leg_dist = delta_point.norm();
                double direction = atan2(delta_point[1], delta_point[0]);
                Eigen::Vector3d mid_pose(mid_point[0], mid_point[1], direction);

                if (inAbsRange(leg_dist, rack_para_.rack_leg_center_width, rack_para_.rack_leg_diameter)) {
                    if (isValid(mid_pose, rack_para_.rack_leg_center_length / 2.)) {
                        leg_pairs.emplace_back();
                        leg_pairs.back().mid_pose = mid_pose;
                        leg_pairs.back().first_leg=legs_cluster[i].center_point;
                        leg_pairs.back().second_leg = legs_cluster[j].center_point;
                        leg_pairs.back().angle = mid_pose[2] - M_PI_2;
                    }
                } else if (inAbsRange(leg_dist, rack_para_.rack_leg_center_length, rack_para_.rack_leg_diameter)) {
                    if (isValid(mid_pose, rack_para_.rack_leg_center_width / 2.)) {
                        leg_pairs.emplace_back();
                        leg_pairs.back().first_leg = legs_cluster[i].center_point;
                        leg_pairs.back().second_leg = legs_cluster[j].center_point;
                        leg_pairs.back().mid_pose = mid_pose;
                        leg_pairs.back().angle = mid_pose[2];
                    }
                }
            }
        }
        if (leg_pairs.size()) {
            return true;
        }
        return false;
    }

    bool isValid(const Eigen::Vector3d &mid_point, double half_length) {
        Eigen::Affine2d laser_tf(
                Eigen::Translation2d(laser_pose_[0], laser_pose_[1]) * Eigen::Rotation2Dd(laser_pose_[2]));

        Eigen::Affine2d mid_point_tf(
                Eigen::Translation2d(mid_point[0], mid_point[1]) * Eigen::Rotation2Dd(mid_point[2]));

        auto center_point = laser_tf * mid_point_tf * Eigen::Vector2d(0, half_length);
        double dist = center_point.norm();
        if (inRange(dist, 0, rack_para_.rack_leg_diameter)) {
            return true;
        }
        return false;
    }

    void creatLegPara(std::vector<ClustterInfo> &leg_cluster, std::vector<std::vector<Eigen::Vector2d> > &multi_legs) {
        if (leg_cluster.size() < 2) {
            LOG(WARNING) << "only find one cluster! will return false!";
            return;
        }

        std::vector<LegPair> leg_pairs;
        if (findValidLegs(leg_cluster,leg_pairs)) {
            int last_size = leg_pairs.size();

            updateMotionPrediction(leg_pairs);//用来更新货架位姿观测

            if (last_size != leg_pairs.size()) {
                LOG(INFO) << "new size:" << last_size << "," << leg_pairs.size();
            }
            for (auto &leg_pair:leg_pairs) {
                multi_legs.emplace_back();
                auto &legs = multi_legs.back();

                auto first_leg = leg_pair.first_leg;
                auto second_leg = leg_pair.second_leg;
                auto corner_point = (first_leg + second_leg) / 2.0;
                auto delta_point = (second_leg - first_leg);
                double leg_dist = delta_point.norm();
                double direction = atan2(delta_point[1], delta_point[0]);
                Eigen::Affine2d corner_tf(Eigen::Translation2d(corner_point) * Eigen::Rotation2Dd(direction));

                compute_info.rack_center_pose[0] = corner_point[0];
                compute_info.rack_center_pose[1] = corner_point[1];
                compute_info.rack_center_pose[2] = direction;

                Eigen::Vector2d local_first_point, local_second_point, local_third_point, local_forth_point;
                if (inAbsRange(leg_dist, rack_para_.rack_leg_center_width, rack_para_.rack_leg_diameter)) {
                    local_first_point[0] = rack_para_.rack_leg_center_width / 2.;
                    local_first_point[1] = 0;
                    local_second_point[0] = rack_para_.rack_leg_center_width / 2.0;
                    local_second_point[1] = rack_para_.rack_leg_center_length;
                    local_third_point[0] = -rack_para_.rack_leg_center_width / 2.;
                    local_third_point[1] = rack_para_.rack_leg_center_length;
                    local_forth_point[0] = -rack_para_.rack_leg_center_width / 2.;
                    local_forth_point[1] = 0;
                } else {
                    local_first_point[0] = rack_para_.rack_leg_center_length / 2.;
                    local_first_point[1] = 0;
                    local_second_point[0] = rack_para_.rack_leg_center_length / 2.0;
                    local_second_point[1] = rack_para_.rack_leg_center_width;
                    local_third_point[0] = -rack_para_.rack_leg_center_length / 2.;
                    local_third_point[1] = rack_para_.rack_leg_center_width;
                    local_forth_point[0] = -rack_para_.rack_leg_center_length / 2.;
                    local_forth_point[1] = 0;

                }


                Eigen::Vector2d first_point = corner_tf * local_first_point;
                Eigen::Vector2d second_point = corner_tf * local_second_point;
                Eigen::Vector2d third_point = corner_tf * local_third_point;
                Eigen::Vector2d forth_point = corner_tf * local_forth_point;
                legs.push_back(first_point);
                legs.push_back(second_point);
                legs.push_back(third_point);
                legs.push_back(forth_point);
            }

        } else {
            LOG(INFO) << "cannot find leg!";
        }

    }


    bool nearWithCluster(const std::vector<RangeInfo> &points, const RangeInfo &cand_point, double angle_offset) {
        double delta_angle = points.back().angle - cand_point.angle;
        normalizeAngle(delta_angle);
        if (fabs(delta_angle) <= angle_offset) {
            return true;
        }
        return false;
    }

    struct LegInfo {
        double laser_angle;
        double laser_dist;
    };

    struct LegRangeIndex {
        LegRangeIndex(const int &ind, const double &dis) : index(ind), dist(dis) {

        }

        unsigned int index = 0;
        double dist = 0.0f;
    };

    inline double normalizeAngle(const double &angle) {
        auto new_angle = fmod(angle, 2.0f * M_PI);
        if (new_angle >= M_PI) {
            new_angle -= 2.0f * M_PI;
        } else if (new_angle < -M_PI) {
            new_angle += 2.0f * M_PI;
        }
        return new_angle;
    }

    void filterNoiseLeg(ScanType &scan, const LegInfo &leg) {
        // 计算当前腿中心位置的index
        auto laser_leg_index = (unsigned long) lround((leg.laser_angle - scan->angle_min) / scan->angle_increment);
        // 计算搜索范围的index
        auto delta_leg_index = (unsigned long) lround(rack_para_.rack_leg_search_thresh / scan->angle_increment);
        // 计算拖尾范围对应的index.一般情况下,拖尾不超过1°
        auto delta_tail_noise_index = (unsigned long) lround(
                rack_para_.tailing_noise_angle_thresh / scan->angle_increment);

        const auto range_max_index = scan->ranges.size() - 1;

        // 计算搜索范围最小index
        auto index_min = laser_leg_index - delta_leg_index;
        index_min = (index_min < 0) ? 0 : index_min;

        // 计算搜索范围最大index
        auto index_max = laser_leg_index + delta_leg_index;
        index_max = (index_max > range_max_index) ? range_max_index : index_max;

        std::vector<LegRangeIndex> leg_points;
        // 搜索得到的leg具体index
        LegRangeIndex min_range(range_max_index, 0.0f), max_range(0, 0.0);

        auto &ranges = scan->ranges;
        // 在搜索范围内查找leg的具体位置
        for (auto i = index_min; i < index_max; i++) {
            // 如果某个点的range小于leg（考虑直径），则认为这个点就是leg的一部分
            if (inRange(ranges[i], leg.laser_dist, rack_para_.rack_leg_diameter)) {
                LegRangeIndex leg_point(i, ranges[i]);

                // 更新搜索到的leg具体index范围
                if (min_range.index > i) {
                    min_range = leg_point;
                }
                if (max_range.index < i) {
                    max_range = leg_point;
                }
                leg_points.push_back(leg_point);
            }
        }

        if (leg_points.empty()) {
            // 如果没有找到任何点，那么说明没有leg，不继续处理
            return;
        }

        // 将搜索到的leg范围加上拖尾点范围，得到需要滤掉的准确index范围
        index_min = min_range.index - delta_tail_noise_index;
        index_min = (index_min < 0) ? 0 : index_min;

        index_max = max_range.index + delta_tail_noise_index;
        index_max = (index_max > range_max_index) ? range_max_index : index_max;

        assert(index_min <= index_max);

        // 如果某个点的距离大于leg实际距离+下面的阈值，那么不认为其是leg
        const double MAX_LEG_RANGE_THERSH = 1.0; // 单位m

        // 将搜索到的leg范围内的点range全部清零
        for (auto i = index_min; i < index_max; ++i) {
            if (inRange(ranges[i], leg.laser_dist, MAX_LEG_RANGE_THERSH)) {
                scan->ranges[i] = 0; // 直接修改参数scan->ranges
            }
        }

    }

    void computeRegionInfo(ScanType &scan) {
        range_region.clear();
        range_region.resize(scan->ranges.size());
        int leg_size = laser_legs_.size();
        double min_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = min_angle;
        double radius_thresh = sqrt(rack_para_.rack_leg_center_length * rack_para_.rack_leg_center_length +
                                    rack_para_.rack_leg_center_width * rack_para_.rack_leg_center_width) +
                               rack_para_.rack_radius_offset;//计算允许过滤的最远距离

        for (auto &range:range_region) {//将所有角度的range均计算出过滤的最远距离
            double tmp_range = 100000.0;
            //利用极坐标直线公式:r((y1-y0)*cos(angle)-(x1-x0)*sin(angle))=x0y1-x1y0,
            // 计算当前角度方向与货架四个线段的角点,从而得到radius
            for (int i = 1; i < leg_size + 1; ++i) {
                int index_1 = i % leg_size;
                int index_0 = (i - 1) % leg_size;

                double k_value = (laser_legs_[index_1].y() - laser_legs_[index_0].y()) * cos(angle) -
                                 (laser_legs_[index_1].x() - laser_legs_[index_0].x()) * sin(angle);
                double c = laser_legs_[index_0].x() * laser_legs_[index_1].y() -
                           laser_legs_[index_1].x() * laser_legs_[index_0].y();
                if (k_value != 0.0) {
                    double radius = c / k_value;
                    if (radius > 0) {
                        if (tmp_range > radius) {
                            tmp_range = radius;
                        }
                    }
                }
            }
            if (tmp_range < radius_thresh) {
                range = tmp_range + rack_para_.rack_radius_offset;
            } else {
                LOG(WARNING) << "the range is err!" << tmp_range;
                range = tmp_range;
            }

            angle += angle_incre;
        }
    }

    void computeRegionInfo(ScanType &scan, std::vector<Eigen::Vector2d> &legs) {
//        range_region.clear();
        range_region.resize(scan->ranges.size());
        int leg_size = legs.size();
//        LOG(INFO) << "leg size:" << leg_size;
        double min_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = min_angle;


        double radius_thresh = sqrt(rack_para_.rack_leg_center_length * rack_para_.rack_leg_center_length +
                                    rack_para_.rack_leg_center_width * rack_para_.rack_leg_center_width) +
                               rack_para_.rack_radius_offset;//计算允许过滤的最远距离


        for (auto &range:range_region) {//将所有角度的range均计算出过滤的最远距离
            double tmp_range = 100000.0;
            //利用极坐标直线公式:r((y1-y0)*cos(angle)-(x1-x0)*sin(angle))=x0y1-x1y0,
            // 计算当前角度方向与货架四个线段的角点,从而得到radius
            for (int i = 1; i < leg_size + 1; ++i) {
                int index_1 = i % leg_size;
                int index_0 = (i - 1) % leg_size;

                double k_value = (legs[index_1].y() - legs[index_0].y()) * cos(angle) -
                                 (legs[index_1].x() - legs[index_0].x()) * sin(angle);
                double c = legs[index_0].x() * legs[index_1].y() - legs[index_1].x() * legs[index_0].y();
                if (k_value != 0.0) {
                    double radius = c / k_value;
                    if (inLine(legs[index_0], legs[index_1], radius, angle)) {
                        if (radius > 0) {
                            if (tmp_range > radius_thresh) {
                                tmp_range = radius;
                            }else if (tmp_range < radius) {
                                tmp_range = radius;
                            }
                        }
                    }
                }
            }
            if (tmp_range < radius_thresh) {
                range = tmp_range ;
            } else {
//                LOG(WARNING) << "the range is err!" << tmp_range;
                range = 0;
            }

            angle += angle_incre;
        }
    }

    bool inLine(const Eigen::Vector2d &first_point, const Eigen::Vector2d &second_point, double range, double &angle) {
        Eigen::Vector2d center_point(range * cos(angle), range * sin(angle));
        const Eigen::Vector2d &delta_1 = center_point - first_point;
        const Eigen::Vector2d &delta_2 = center_point - second_point;
        if (delta_1.dot(delta_2) <= 0.) {
            return true;//在直线上
        }
        return false;
    }

    bool inRegion(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &legs) {
        if (legs.size() != 4) {
            LOG(INFO) << "cannot charge!";
            return false;
        }
        double first_angle = computeAngle(legs[0], legs[1], point);
        if (first_angle < 0.0) {
            return true;
        }
        double second_angle = computeAngle(legs[1], legs[2], point);
        if (second_angle < 0.0) {
            return true;
        }
        double third_angle = computeAngle(legs[2], legs[3], point);
        if (third_angle < 0.0) {
            return true;
        }
        double forth_angle = computeAngle(legs[3], legs[0], point);
        if (forth_angle < 0.0) {
            return true;
        }
        auto sum_angle = first_angle + second_angle + third_angle + forth_angle;
        auto delta_angle = fabs(sum_angle - 2.0f * M_PI);
        if (delta_angle < 0.01f) {
            return true;
        }
        return false;
    }

    double computeAngle(const Eigen::Vector2d &first_point, const Eigen::Vector2d &secont_point,
                        const Eigen::Vector2d &corner_point) {
        auto first_vector = first_point - corner_point;
        auto second_vector = secont_point - corner_point;
        auto first_dist = first_vector.norm();
        auto second_dist = second_vector.norm();
        if (first_dist < 0.005) {
            return -1.0;
        }
        if (second_dist < 0.005) {
            return -1.0;
        }
        auto cos_angle = first_vector.dot(second_vector) / (first_dist * second_dist);
        auto angle = acosf(cos_angle);
        return angle;
    }

    void computeCircleInfo(ScanType &scan) {
        rack_circle.clear();
        rack_circle.resize(scan->ranges.size());
        LOG(INFO) << "begin to compute the rack info!";
        double min_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = min_angle;
        double radius_thresh = (sqrt(rack_para_.rack_leg_center_length * rack_para_.rack_leg_center_length +
                                     rack_para_.rack_leg_center_width * rack_para_.rack_leg_center_width)) / 2.0;//计算允许过滤的最远距离

        Eigen::Affine2f laser_tf(
                Eigen::Translation2f(laser_pose_[0], laser_pose_[1]) * Eigen::Rotation2Df(laser_pose_[2]));
        Eigen::Vector2f center_pose = laser_tf.inverse() * Eigen::Vector2f(0, 0);
        double a_0 = center_pose[0];
        double b_0 = center_pose[1];

        double r_0_2 = a_0 * a_0 + b_0 * b_0;
        double r_0 = sqrt(r_0_2);
        double phi = atan2(b_0, a_0);
        LOG(INFO) << "radius:" << radius_thresh << "," << a_0 << "," << b_0 << "," << phi;
        for (auto &range:rack_circle) {//将所有角度的range均计算出过滤的最远距离
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
//                LOG(INFO) << "range:" << range_1 << "," << range_2 << "," << angle << "," << range;
                if (range < 0) {
                    LOG(WARNING) << "the range is err!" << range;
                    range = 0;
                }
            }

            angle += angle_incre;
        }
    }

    void filterScanByRegion(ScanType &scan) {
        if (!range_region.size()) {
            computeRegionInfo(scan);
        }
        int range_size = scan->ranges.size();
        auto &ranges = scan->ranges;
        for (int i = 0; i < range_size; ++i) {
            if (ranges[i] < range_region[i]) {
                ranges[i] = 0;
            }
        }//将矩形内的所有range去除.
    }

    void filterScanByRectangle(const std::vector<Eigen::Vector2d> &legs, ScanType &scan) {
        double angle_min = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = angle_min;
        Eigen::Vector2d coord;
        for (auto &range:scan->ranges) {
            coord[0] = range * cos(angle);
            coord[1] = range * sin(angle);
            angle += angle_incre;
            if (inRegion(coord, legs)) {
                range = 0.0;
            }
        }
    }

    inline bool inRange(double dist, double dist_thresh, double offset) {
        return dist < (dist_thresh + offset);
    }

    inline bool inAbsRange(double dist, double dist_thresh, double offset) {
        return ((dist <= (dist_thresh + offset)) && (dist >= (dist_thresh - offset)));
    }

    inline bool inMaxAndMinRange(double dist,double dist_thresh,double max_offset,double min_offset){
        return ((dist <= (dist_thresh + max_offset)) && (dist >= (dist_thresh - min_offset)));
    }

    void updateMotionPrediction(std::vector<LegPair>& leg_pairs) {
        if (motion_prediction.isInitialized()) {
            std::vector<double> yaws;
            for (auto &pairs:leg_pairs) {
                yaws.push_back(pairs.angle);
            }

            std::vector<LegPair> right_pairs;
            int count = 0;
            for (auto &yaw:yaws) {
                if (motion_prediction.isNear(yaw, 5 * M_PI / 180.0)) {
                    right_pairs.push_back(leg_pairs[count]);
                }
                count++;
            }//必须先进行寻找,再进行最近yaw搜索,防止出现局部最优.该功能不足够鲁邦,所以,不能过于轻信.

            auto index = motion_prediction.findNearestIndex(yaws);
            if (index >= 0) {
                motion_prediction.pushBackAngle(yaws[index]);
            }
            if (!motion_prediction.isRotate(5 * M_PI / 180)) {
            }else {
                LOG(INFO) << "detect rotate!";
            }

            if (right_pairs.size()) {
                leg_pairs.swap(right_pairs);
            }

        }else {
            if (leg_pairs.size() == 1) {
                motion_prediction.initialize(leg_pairs[0].angle);
            }
        }
    }

private:
    RackPara rack_para_; // 货架leg的相关参数

    Eigen::Vector3d laser_pose_; // 雷达安装位置（相对于运动中心）

    // 雷达角度范围
    double laser_angle_min_ = -2.0f;
    double laser_angle_max_ = 2.0f;

    std::vector<LegInfo> leg_infos_;

    std::vector<Eigen::Vector2f> laser_legs_;

    std::vector<double> range_region;
    std::vector<double> rack_circle;//用作识别任意雷达点上圆曲线上

    ComputeInfo compute_info;
    rack::MotionPrediction motion_prediction;
};


}


#endif //SROS_RACK_LEGS_FILTER_HPP
