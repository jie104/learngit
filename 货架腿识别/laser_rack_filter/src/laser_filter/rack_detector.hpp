//
// Created by lfc on 19-1-17.
//

#ifndef PROJECT_RACK_DETECTOR_HPP
#define PROJECT_RACK_DETECTOR_HPP

#include <glog/logging.h>
#include <Eigen/Dense>
#include "rack_para.hpp"

namespace rack{
//typedef sensor_msgs::LaserScanPtr ScanType;


class RackSearchInfo{
public:
    RackSearchInfo(rack::RackInfo_Ptr &info):rack_info(info) {
        computeRackInfo(rack_info);
    }

    virtual ~RackSearchInfo(){

    }

    void computeRackInfo(rack::RackInfo_Ptr &info_ptr) {
        //找到面积最小的尺寸面，作为特征尺寸。特征尺寸用来识别货架
        auto &info = *info_ptr;
        auto &para = info.rack_para;
        if (info.leg_groups.size()) {
            auto min_square = info.leg_groups[0].length * info.leg_groups[0].width;
            eigen_height = info.leg_groups[0].length / 2.0;//一半，TODO，为什么是一半
            eigen_width = info.leg_groups[0].width;

            para.rack_leg_diameter = info.leg_d;
            para.rack_length = info.leg_groups[0].length;
            para.rack_width = info.leg_groups[0].width;
            for (auto &region:info.leg_groups) {
                auto curr_square = region.width * region.length;
                if (min_square > curr_square) {
                    min_square = curr_square;
                    eigen_height = region.length/2.0;
                    eigen_width = region.width;
                }
                if (para.rack_length < region.length) {
                    para.rack_length = region.length;
                }
                if (para.rack_width < region.width) {
                    para.rack_width = region.width;
                }
            }
            LOG(INFO) << "para info:" << para.rack_length << "," << para.rack_width;
        }else {
            LOG(INFO) << "rack is wrong! cannot detect the rack!";
            para.rack_leg_diameter = 0.0;
            para.rack_length = 0.0;
            para.rack_width = 0.0;
        }
    }
    rack::RackInfo_Ptr rack_info;
    double eigen_height = 0;//特征高度
    double eigen_width = 0;//特征宽度
};

template <class ScanType>
class RackDetector {
public:

    struct RadarPointInfo{
        Eigen::Vector2d point;
    };

    struct ClusterPoint{
        int min_index;
        float min_index_range;
        int max_index;
        float max_index_range;
        std::vector<RadarPointInfo> infos;

        Eigen::Vector2d computeCenterPoint() {
            Eigen::Vector2d center_point;
            center_point.setZero();
            for (auto &info:infos) {
                center_point += info.point;
            }
            if (infos.size()) {
                center_point = center_point * (1/(double) infos.size());
            }
            return center_point;
        }
    };
    typedef std::shared_ptr<ClusterPoint> ClusterPoint_Ptr;

    RackDetector(rack::InstallPara& para,std::vector<rack::RackInfo_Ptr>& infos):install_para_(para) {
        search_infos_.clear();
        min_width_thresh_ = 10.0;
        max_width_thresh_ = 0.0;
        for (auto &info:infos) {
            search_infos_.emplace_back(RackSearchInfo(info));
            double curr_min_width = search_infos_.back().eigen_width - search_infos_.back().rack_info->leg_d * 2.0;
            double curr_max_width = search_infos_.back().eigen_width + search_infos_.back().rack_info->leg_d * 2.0;
            min_width_thresh_ = min_width_thresh_ < curr_min_width ? min_width_thresh_ : curr_min_width;
            max_width_thresh_ = max_width_thresh_ > curr_max_width ? max_width_thresh_ : curr_max_width;
        }
        LOG(INFO) << "min width thresh:" << min_width_thresh_ << ",max:" << max_width_thresh_ << "," << height_thresh;
    }

    bool detectRack(ScanType& scan,RackInfo_Ptr& para,double curr_direction = 0.0){
        std::vector<Eigen::Vector2d> legs;
        return detectRack(scan, para, legs, curr_direction);
    }


    bool detectRack(ScanType& scan,RackInfo_Ptr& para,std::vector<Eigen::Vector2d> &legs,double curr_direction = 0.0){
        if(check_rack_circle_.empty()) {
            computeCircleInfo(scan);
        }
        std::vector<bool> points_state(scan->ranges.size(),true);
        filterOutBoarderPoint(scan, install_para_);
        filterLowIntenPoint(scan);
        filterScanTrailingPoint(scan,points_state,2,0.02);
        std::vector<ClusterPoint_Ptr> clusters;
        extractClusters(scan, points_state, clusters);
        dilateClusters(scan, clusters);
        std::vector<Eigen::Vector2d> rack_legs;
        for (auto &cluster:clusters) {
            legs.emplace_back(cluster->computeCenterPoint());
        }
        convertToRackLegs(legs, rack_legs, curr_direction);
        if (legs.size() < 2) {
            LOG(INFO) << "cannot detect rack!"<<legs.size();
        }
        std::vector<RackLegGroup> groups;
        findRackGroups(rack_legs, groups);
        return findRackInfo(groups, para);
    }

private:
    void findRackGroups(std::vector<Eigen::Vector2d> &rack_legs,std::vector<RackLegGroup> &groups){
        Eigen::Vector2d unit_vector(cos(0), sin(0));
        std::map<double,RackLegGroup> group_map;
        int rack_size = rack_legs.size();
        LOG(INFO) << "rack size:" << rack_size;
        for (int i = 0; i < rack_size; ++i) {
            for (int j = i + 1; j < rack_size; ++j) {
                auto &first_leg = rack_legs[i];
                auto &second_leg = rack_legs[j];
                Eigen::Vector2d delta_vector = second_leg - first_leg;
                double leg_width = delta_vector.norm();
                LOG(INFO) << "leg width:" << leg_width;
                if (leg_width > min_width_thresh_ && leg_width < max_width_thresh_) {
                    auto center = (second_leg + first_leg) * 0.5;
                    auto cos_angle = delta_vector.dot(unit_vector) / leg_width;
                    double height = fabs(unit_vector[0] * center[1] - unit_vector[1] * center[0]);
                    LOG(INFO) << "width:" << leg_width << "," << i << "," << rack_size << "," << height << "," <<
                    height_thresh << "," << min_width_thresh_;
//                    LOG(INFO) << "cos angle:" << fabs(acos(cos_angle)) * 180.0 / M_PI - 90.0 << "height:" << height <<
//                    "," << leg_width << "," << min_width_thresh_ << "," << max_width_thresh_<<","<<fabs(unit_vector.dot(center));
                    if (height < height_thresh) {
//                        groups.emplace_back();
                        RackLegGroup group;
                        group.length = fabs(unit_vector.dot(center));
                        group.width = leg_width;
                        group_map[height] = group;
                    }
                }
            }
        }
        deleteRepeatLegGroup(group_map, groups, 0.01);

    }

    void deleteRepeatLegGroup(std::map<double,RackLegGroup> &group_map,std::vector<RackLegGroup> &groups,double delta_dist_thresh) {
        int group_size = group_map.size();
        groups.resize(group_size);
        int index = 0;
        for (auto &group:group_map) {
            if (index < group_size) {
                groups[index] = group.second;
                index++;
            }
        }


        std::vector<bool> group_states(group_size, true);
        for (int i = 0; i < group_size; ++i) {
            for (int j = i + 1; j < group_size; ++j) {
                if (group_states[i] && group_states[j]) {
                    double width_err = fabs(groups[i].width - groups[j].width);
                    double length_err = fabs(groups[i].length - groups[j].length);
                    if (width_err <= delta_dist_thresh && length_err <= delta_dist_thresh) {
                        group_states[j] = false;
                    }
                }
            }
        }
        std::vector<RackLegGroup> bk_groups;
        bk_groups.reserve(group_size);
        for (int k = 0; k < group_size; ++k) {
            if (group_states[k]) {
                bk_groups.push_back(groups[k]);
            }
        }
        groups.swap(bk_groups);
//        LOG(INFO) << "group size:" << group_size << "new size:" << groups.size();
    }

    bool findRackInfo(std::vector<RackLegGroup> &groups,RackInfo_Ptr& para) {
        if (groups.empty()) {
            LOG(INFO) << "cannot get the rack! group is empty!";
            return false;
        }
        std::sort(groups.begin(), groups.end(),
                  [](const RackLegGroup &lhs, const RackLegGroup &rhs) { return lhs.length*lhs.width < rhs.length*rhs.width; });
        double para_ma_dist = max_ma_dist;
        Eigen::Matrix2d err_cov = Eigen::Matrix2d::Identity();
        err_cov = err_cov * (rack_err_thresh * rack_err_thresh);
        for(auto& group:groups) {
//            LOG(INFO) << "group:" << group.width << "," << group.length;
        }
        for (auto &group:groups) {
            for (auto &search:search_infos_) {
                double width_err = fabs(search.eigen_width - group.width);
                double length_err = fabs(search.eigen_height - group.length + search.rack_info->leg_d / 2.0);
                double leg_d = search.rack_info->leg_d;
                if (width_err < (rack_err_thresh + leg_d/2.0) && length_err < (rack_err_thresh + leg_d/2.0)) {
                    Eigen::Vector2d err_mat(width_err, length_err);
                    auto err_info =
                            (err_cov + Eigen::Matrix2d::Identity() * (leg_d * leg_d)).inverse();

                    auto ma_dist = err_mat.transpose() * err_info * err_mat;
                    LOG(INFO) << "para ma dist:" << para_ma_dist << "ma dist:" << ma_dist << "," << width_err << "," <<
                    length_err << "," << search.eigen_width << "," << search.eigen_height << "," << group.width <<
                    "," << group.length;
                    if (para_ma_dist > ma_dist) {
                        para_ma_dist = ma_dist;
                        para = search.rack_info;
                    }
                }
            }
            if (para_ma_dist < max_ma_dist) {//优先选择面积最小的货架识别
                auto &rack = para->rack_para;
                LOG(INFO) << "para:" << rack.rack_length << "," << rack.rack_width << "," << "size:" <<
                search_infos_.size();
                return true;
            }
        }
        if (para_ma_dist < max_ma_dist) {
            auto &rack = para->rack_para;
            LOG(INFO) << "para:" << rack.rack_length << "," << rack.rack_width << "," << "size:" <<
            search_infos_.size();
            return true;
        }
        return false;
    }

    void convertToRackLegs(std::vector<Eigen::Vector2d>& legs,std::vector<Eigen::Vector2d>& rack_legs,double curr_direction){
        Eigen::Affine2d rotation_tf(Eigen::Translation2d(Eigen::Vector2d::Zero())*Eigen::Rotation2Dd(curr_direction));
        Eigen::Affine2d center_tf(
                Eigen::Translation2d(install_para_.laser_coord_x, install_para_.laser_coord_y) * Eigen::Rotation2Dd(install_para_.laser_coord_yaw));
        for (auto &leg:legs) {
            rack_legs.push_back(rotation_tf.inverse() * center_tf * leg);
        }
    }

    double getMaxRackDist(){
        double max_dist = 0.0;
        for (auto &search_info:search_infos_) {
            double curr_dist = std::hypot(search_info.eigen_height, search_info.eigen_width);   //求直角三角形斜边长
            if (max_dist < curr_dist) {
                max_dist = curr_dist;
            }
        }
        return max_dist;
    }

    void computeCircleInfo(ScanType &scan) {
        check_rack_circle_.clear();
        check_rack_circle_.resize(scan->ranges.size());
//        LOG(INFO) << "begin to compute the rack info!";
        double min_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        double angle = min_angle;
        double radius_thresh = getMaxRackDist() + check_dist_offset;//计算允许过滤的最远距离

        //将以雷达安装位置为原点的坐标系上的点云转化到转化到标准坐标系上，即原点(0,0),即将雷达坐标系的点云转到车体中心
        Eigen::Affine2f laser_tf(
                Eigen::Translation2f(install_para_.laser_coord_x, install_para_.laser_coord_y) * Eigen::Rotation2Df(install_para_.laser_coord_yaw));
        Eigen::Vector2f center_pose = laser_tf.inverse() * Eigen::Vector2f(0, 0);   //车体中心在雷达坐标系下的坐标
        double a_0 = center_pose[0];
        double b_0 = center_pose[1];

        double r_0_2 = a_0 * a_0 + b_0 * b_0;
        double r_0 = sqrt(r_0_2);
        double  phi = atan2(b_0, a_0);
//        LOG(INFO) << "radius:" << radius_thresh << "," << a_0 << "," << b_0 << "," << phi;
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

    void filterLowIntenPoint(ScanType& scan){
        int scan_size = scan->ranges.size();
        auto &ranges = scan->ranges;
        auto &intens = scan->intensities;

        for (int i = 0; i < scan_size; ++i) {
            if(ranges[i]<check_rack_circle_[i]) {//判断当前range是否在要进行判断的范围之内
                if (intens[i] < low_inten_thresh) {
                    scan->ranges[i] = 0;
                }
            }
        }
    }

    void filterScanTrailingPoint(ScanType& scan,std::vector<bool> &points_state,int step = 2,double min_thresh = 0.02) {//注意，这里如果拖尾部分有一个点被滤掉了，那其他点则无法识别出为拖尾点了，因为，这里有距离判断
        auto& ranges = scan->ranges;

        double cos_increment = cos(scan->angle_increment * (double) step * 2.0);
        double theta_thresh = sin((double) scan->angle_increment * (double) step * 2.0) / sin(0.17);//临界值,用于识别断点

        int scan_size = ranges.size() - step;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 0 || ranges[i] > check_rack_circle_[i]) {
                continue;
            }

            double dist_1 = std::sqrt(ranges[i+step] * ranges[i+step] + ranges[i] * ranges[i] -
                                      2 * ranges[i+step] * ranges[i] * cos_increment);

            double range_thresh_1 = ranges[i + step] * theta_thresh + min_thresh;
            if(dist_1 > range_thresh_1) {
                int remove_gap = step;
                for (int j = -remove_gap; j <= remove_gap; ++j) {
                    points_state[i+j] = false;
                }
            }
        }
    }


    void extractClusters(ScanType &scan,std::vector<bool> &points_state,std::vector<ClusterPoint_Ptr> &clusters){
        auto& ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        int index = 0;
        clusters.clear();
        clusters.emplace_back(new (ClusterPoint));
        for (auto &range:ranges) {
            if (range < range_min || range > check_rack_circle_[index]||!points_state[index]) {
                if (clusters.back()->infos.empty()) {

                }else if(clusters.back()->infos.size()<= min_cluster_thresh){//孤立点
                    clusters.back()->infos.clear();
                }else {
                    clusters.emplace_back(new (ClusterPoint));
                }
            }else {
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


    void dilateClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &clusters){
        double increment = scan->angle_increment;
        auto &ranges = scan->ranges;
        int max_size = scan->ranges.size() - 1;
        int delta_trailing_size = round(dilate_cluster_angle_offset / scan->angle_increment);
        for(auto& cluster:clusters){
            auto curr_trailing_size =
                    cluster->min_index - delta_trailing_size > 0 ? delta_trailing_size : cluster->min_index;
            dilateClusterBoarder(scan, cluster, cluster->min_index, curr_trailing_size, -1);
            curr_trailing_size = cluster->max_index + delta_trailing_size > max_size ? max_size - cluster->max_index
                                                                                     : delta_trailing_size;
            dilateClusterBoarder(scan, cluster, cluster->max_index, curr_trailing_size, 1);
        }
    }

    void dilateClusterBoarder(ScanType &scan,ClusterPoint_Ptr& cluster,int& cluster_index,int dilate_size,int direction = 1){
        const int beg_index = cluster_index;
        auto &ranges = scan->ranges;
        double min_angle = scan->angle_min;
        double increment = scan->angle_increment;
        auto ref_range = ranges[beg_index];
        for (int i = 1; i < dilate_size; ++i) {
            auto first_edge_index = beg_index + i * direction;
            auto range = ranges[first_edge_index];
            auto delta_dist =
                    range * range + ref_range * ref_range - 2.0 * range * ref_range * cos(i * increment);
            if (delta_dist <= dilate_cluster_dist_thresh * dilate_cluster_dist_thresh) {
                cluster_index = first_edge_index;
                cluster->infos.emplace_back();
                auto &info = cluster->infos.back();
                double curr_angle = first_edge_index * increment + min_angle;
                info.point = Eigen::Vector2d(range * cos(curr_angle), range * sin(curr_angle));
            }
        }
    }



    void filterOutBoarderPoint(ScanType& scan,InstallPara& install_para){
        auto& ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        for (auto &range:ranges) {
            if (range < range_min || range > range_max) {
                range = 0;
            }else {
                if (curr_angle < install_para.laser_angle_min || curr_angle > install_para.laser_angle_max) {
                    range = 0;
                }
            }
            curr_angle += angle_incre;
        }
    }

private:
    std::vector<RackSearchInfo> search_infos_;
    InstallPara install_para_;
    std::vector<double> check_rack_circle_;
    const double check_dist_offset = 0.2;
    const double min_trailing_thresh = 0.03;
    const double low_inten_thresh = 50;
    const int min_cluster_thresh = 3;
    const double cos_thresh = cos(85 * M_PI / 180.0);
    const double height_thresh = 0.08;
    const double rack_err_thresh = 0.05;
    const double max_ma_dist = 2.0;
    const double dilate_cluster_angle_offset = 1.0*M_PI/180.0;
    const double dilate_cluster_dist_thresh = 0.03;

    double min_width_thresh_ = 0.5;
    double max_width_thresh_ = 1.0;

};


}


#endif //PROJECT_RACK_DETECTOR_HPP
