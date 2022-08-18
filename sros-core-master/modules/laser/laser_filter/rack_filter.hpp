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

class BaseRegion{
 public:
    enum RegionType{
        TYPE_RECTANGLE = 1,
        TYPE_TRIANGLE = 2,
        TYPE_CIRCLE = 3,
    };
    BaseRegion(RegionType type):type_(type){

    }

    virtual ~BaseRegion(){

    }

    RegionType type(){
        return type_;
    }

    virtual bool inRectangle(const double &coord_x,const double &coord_y) const = 0;

    RegionType type_;
};

class Rectangle :public BaseRegion{
 public:
    Rectangle():BaseRegion(BaseRegion::TYPE_RECTANGLE){

    }

    virtual bool inRectangle(const double &coord_x,const double &coord_y) const{
        if (coord_x >= min_x && coord_x <= max_x) {
            if (coord_y >= min_y && coord_y <= max_y) {
                return true;
            }
        }
        return false;
    }
    double min_y = 0.6 / 2;
    double max_y = -0.6 / 2;
    double min_x = -1.06 / 2.0;
    double max_x = 1.06 / 2.0;
};

class Triangle:public BaseRegion{
 public:
    Triangle():BaseRegion(BaseRegion::TYPE_TRIANGLE){

    }

    void computeEgdeVector(){
        vect_ab = second_point - first_point;
        vect_ac = third_point - first_point;
        vect_bc = third_point - second_point;

        cross_ac_ab = cross(vect_ac,vect_ab);
        cross_ab_ac = -cross_ac_ab;
        cross_ba_bc = cross(-vect_ab,vect_bc);
    }

    double cross(const Eigen::Vector2d& first,const Eigen::Vector2d& second)const {
        return first[0] * second[1] - first[1] * second[0];
    }

    virtual bool inRectangle(const double &coord_x,const double &coord_y){
        Eigen::Vector2d point = Eigen::Vector2d(coord_x,coord_y);
        Eigen::Vector2d vect_ap = point - first_point;
        Eigen::Vector2d vect_bp = point - second_point;
        double cross_ap_ab = cross(vect_ap, vect_ab);
        double cross_ap_ac = cross(vect_ap, vect_ac);
        double cross_bp_bc = cross(vect_bp, vect_bc);
        double direct_1 = cross_ap_ab * cross_ac_ab;
        double direct_2 = cross_ap_ac * cross_ab_ac;
        double direct_3 = cross_bp_bc * cross_ba_bc;
        return (direct_1 >= 0) && (direct_2 >= 0) && (direct_3 >= 0);
    }

    Eigen::Vector2d first_point;
    Eigen::Vector2d second_point;
    Eigen::Vector2d third_point;

 private:
    Eigen::Vector2d vect_ab;
    Eigen::Vector2d vect_ac;
    Eigen::Vector2d vect_bc;
    double cross_ac_ab;
    double cross_ab_ac;
    double cross_ba_bc;

};

class Circle:public BaseRegion{
 public:
    Circle():BaseRegion(BaseRegion::TYPE_CIRCLE){

    }
    virtual bool inRectangle(const double &coord_x,const double &coord_y) const{
        double dist = std::hypot(coord_x, coord_y);
        if (dist < radius_max) {
            double angle = atan2(coord_y, coord_x);
            if(angle>=min_theta&&angle<=max_theta){
                return true;
            }
        }
        return false;
    }

    double min_theta;
    double max_theta;
    double radius_max;
 private:

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
        return rectangle_->inRectangle(tmp_x, tmp_y);
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
class BaseRackFilter{
 public:
    enum RackFilterType{
        TYPE_NORMAL_FILTER = 1,
        TYPE_LEGS_FILTER = 2,
    };

    BaseRackFilter(RackFilterType type,InstallPara& install_para):type_(type),install_para_(install_para){
        rack_rectangle_.reset(new Rectangle);
        rack_leg_trailing_angle = install_para_.trailing_angle;
    }

    virtual ~BaseRackFilter(){

    }


    void filterRack(ScanType &scan, double curr_direction = 0.0) {
        std::vector<ClusterPoint_Ptr> rack_clusters;
        filterRack(scan, rack_clusters, curr_direction);
    }

    virtual void computeRackInfo(RackPara &rack) = 0;

    virtual void findRackCluster(std::vector<ClusterPoint_Ptr> &clusters, std::vector<ClusterPoint_Ptr> &rack_clusters,
                                 double direction) = 0;
    RackFilterType type(){

    }

 protected:
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


    void filterRack(ScanType &scan, std::vector<ClusterPoint_Ptr> rack_clusters, double curr_direction = 0.0) {
        if (check_rack_circle_.empty()) {
            computeCircleInfo(scan);
        }
        std::vector<bool> points_state(scan->ranges.size(), true);
        filterOutBoarderPoint(scan, install_para_);
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


    void filterRackClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &rack_clusters) {
        int max_size = scan->ranges.size() - 1;
        int delta_trailing_size = round(rack_leg_trailing_angle / scan->angle_increment);
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

    inline bool inRange(double dist, double dist_thresh, double offset) {
        return dist < (dist_thresh + offset);
    }

    void dilateClusters(ScanType &scan, std::vector<ClusterPoint_Ptr> &clusters) {
        double increment = scan->angle_increment;
        auto &ranges = scan->ranges;
        int max_size = scan->ranges.size() - 1;
        int delta_trailing_size = round(dilate_cluster_angle_offset / scan->angle_increment);
        for (auto &cluster:clusters) {
            auto curr_trailing_size =
                cluster->min_index - delta_trailing_size > 0 ? delta_trailing_size : cluster->min_index;
            dilateClusterBoarder(ranges, increment, cluster->min_index, curr_trailing_size, -1);
            curr_trailing_size = cluster->max_index + delta_trailing_size > max_size ? max_size - cluster->max_index
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
#ifdef ROS_DEBUG_API
    public:
#else
 protected:
#endif

    InstallPara install_para_;
    const double rectangle_angle_step = 2.0 * M_PI / 180.0;
    const double laser_std_err = 0.03;
    std::vector<double> check_rack_circle_;
    const double check_dist_offset = 0.5;
    const double dilate_cluster_angle_offset = 1.5 * M_PI / 180.0;
    double rack_leg_trailing_angle = 2.0 * M_PI / 180.0;
    const double cluster_in_rectangle_thresh = 0.8;
    Rectangle_Ptr rack_rectangle_;
    const double dilate_cluster_dist_thresh = 0.03;

 private:
    RackFilterType type_;
};


template<class ScanType>
class RackLegsFilter:public BaseRackFilter<ScanType>{
 public:
    RackLegsFilter(InstallPara &install_para):BaseRackFilter<ScanType>(BaseRackFilter<ScanType>::TYPE_NORMAL_FILTER,install_para){

    }



    virtual void computeRackInfo(RackPara &rack) {
        double max_x = (rack.rack_length / 2.0 + rack.rack_leg_diameter + this->laser_std_err);
        double min_x = -max_x;
        double max_y = (rack.rack_width / 2.0 + rack.rack_leg_diameter + this->laser_std_err);
        double min_y = -max_y;
        this->rack_rectangle_->max_x = max_x;
        this->rack_rectangle_->max_y = max_y;
        this->rack_rectangle_->min_x = min_x;
        this->rack_rectangle_->min_y = min_y;
        this->check_rack_circle_.clear();

        std::vector<Eigen::Vector2d> rack_leg_center_points;
        double center_x = rack.rack_length / 2.0;
        double center_y = rack.rack_width / 2.0;

        double leg_radius = rack.rack_leg_diameter / 2.0 + this->laser_std_err;
        rack_leg_center_points.push_back(Eigen::Vector2d(center_x, center_y));
        rack_leg_center_points.push_back(Eigen::Vector2d(center_x, -center_y));
        rack_leg_center_points.push_back(Eigen::Vector2d(-center_x, -center_y));
        rack_leg_center_points.push_back(Eigen::Vector2d(-center_x, center_y));
        Eigen::Affine2d laser_tf(Eigen::Translation2d(this->install_para_.laser_coord_x, this->install_para_.laser_coord_y) *
                                 Eigen::Rotation2Dd(this->install_para_.laser_coord_yaw));

        circles.clear();
        int num_times = round(this->install_para_.backlash_angle / this->rectangle_angle_step);
        if (num_times < 1) {
            num_times = 1;
        }
        for (auto &rack_center:rack_leg_center_points) {
            for (int i = -num_times; i <= num_times; ++i) {
                double rotate_delta_angle = double(i) * this->rectangle_angle_step;
                Eigen::Affine2d rotate_angle = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(rotate_delta_angle);

                auto leg = laser_tf.inverse() * rotate_angle * rack_center;
                double leg_dist = leg.norm();
                double delta_angle = asin(leg_radius / leg_dist);
                double center_angle = atan2(leg[1], leg[0]);
                LOG(INFO) << "leg:" << leg[0] << "," << leg[1] << "," << delta_angle << "," << center_angle;
                Circle circle;
                circle.min_theta = center_angle - delta_angle;
                circle.max_theta = center_angle + delta_angle;
                circle.radius_max = leg_dist + leg_radius;
                circles.push_back(circle);
            }
        }
    }

    virtual void findRackCluster(std::vector<ClusterPoint_Ptr> &clusters, std::vector<ClusterPoint_Ptr> &rack_clusters,
                                 double direction) {
        rack_clusters.clear();

        Eigen::Affine2d rotate_tf(Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(-direction));
        Eigen::Affine2d laser_tf(
            Eigen::Translation2d(this->install_para_.laser_coord_x, this->install_para_.laser_coord_y) *
            Eigen::Rotation2Dd(this->install_para_.laser_coord_yaw));
        auto curr_tf = laser_tf.inverse() * rotate_tf * laser_tf;

        for (auto &cluster:clusters) {
            int cluster_in_rack_count = 0;
            for (auto &point:cluster->infos) {
                for (auto &circle:circles) {
                    auto curr_point = curr_tf * point.point;
                    if (circle.inRectangle(curr_point[0],curr_point[1])) {
                        cluster_in_rack_count++;//如果当前点在Center内，则认为该点合理。
                        break;
                    }
                }
            }
            int cluster_size = cluster->infos.size();
            if (cluster_size > 0) {
                double percent = (double) cluster_in_rack_count / (double) cluster_size;
                if (percent > this->cluster_in_rectangle_thresh) {
                    rack_clusters.push_back(cluster);
                }
            }
        }
    }

 private:
    std::vector<Circle> circles;
};


template<class ScanType>
class RackFilter :public BaseRackFilter<ScanType>{
 public:
    RackFilter(InstallPara &install_para):BaseRackFilter<ScanType>(BaseRackFilter<ScanType>::TYPE_NORMAL_FILTER,install_para){
        int num_times = round(this->install_para_.backlash_angle / this->rectangle_angle_step);
        for (int i = -num_times; i <= num_times; ++i) {
            rectangles_.emplace_back();
            rectangles_.back().updateInfo(this->rack_rectangle_, double(i) * this->rectangle_angle_step);
        }
        scan_to_center_tf_ = Eigen::Translation2d(install_para.laser_coord_x, install_para.laser_coord_y) *
                             Eigen::Rotation2Dd(install_para.laser_coord_yaw);
    }


    virtual void computeRackInfo(RackInfo_Ptr &rack_info) {
        auto &rack = rack_info->rack_para;
        computeRackInfo(rack);
    }
    virtual void computeRackInfo(RackPara &rack) {
        double max_x = (rack.rack_length / 2.0 + rack.rack_leg_diameter + this->laser_std_err);
        double min_x = -max_x;
        double max_y = (rack.rack_width / 2.0 + rack.rack_leg_diameter + this->laser_std_err);
        double min_y = -max_y;
        this->rack_rectangle_->max_x = max_x;
        this->rack_rectangle_->max_y = max_y;
        this->rack_rectangle_->min_x = min_x;
        this->rack_rectangle_->min_y = min_y;
        this->check_rack_circle_.clear();
    }

    virtual void findRackCluster(std::vector<ClusterPoint_Ptr> &clusters, std::vector<ClusterPoint_Ptr> &rack_clusters,
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
                if (percent > this->cluster_in_rectangle_thresh) {
                    rack_clusters.push_back(cluster);
                }
            }
        }
    }


 private:

    std::vector<RectangleInfo> rectangles_;
    Eigen::Affine2d scan_to_center_tf_;
};


}


#endif //PROJECT_RACK_FILTER_HPP
