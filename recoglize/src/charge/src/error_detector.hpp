//
// Created by lfc on 19-2-14.
//

#ifndef PROJECT_CHARGING_STATION_DETECTOR_HPP1
#define PROJECT_CHARGING_STATION_DETECTOR_HPP1

#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <stack>
#include "line_solver.hpp"
#include "detector_point.hpp"
#include  <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>

namespace detector {

struct Goodinfo {
    double topline = 1.1;

//  the length of good
    double baseline = 2.112;    //2112mm
    // double waist = 0.1612;
    double waist = 0.1212;
    // double waist_to_base = 0.3648;
    double waist_to_base = 0.3248;
    // double included_angle = 2.0943;
    double included_angle = 2.2943;
};

struct GoodDetectInfo {
    Point center_point;
    Point first_corner;
    Point second_corner;
};

template <class ScanType>
class GoodDetector {

public:
    struct ContinousPoints {
        int start_index;
        int end_index;
    };


    struct Corner {
        Point point;
        int index;
    };

    GoodDetector() {

    }

    bool findGoodStation(ScanType &scan, Goodinfo &info,std::vector<detector::GoodDetectInfo> &detect_infos) {

        // filter(scan,3,0.02);

        std::vector<ContinousPoints> continous_points;
        std::vector<Corner> corners;

        // splitScan(ranges, scan->angle_increment, continous_points);   //找出连续点前后ID的集合
        // convertTrailingToPoint(ranges,scan, continous_points, corners);    //将连续点ID转化为直角坐标
        // if (findGoodStation(ranges,scan, continous_points, info, detect_infos)) {
        //     return true;
        // }
        // return false;
        splitScan(scan,continous_points,25);

        // extractContinueSegment(scan,continous_points,0.04);       
        // extractContinueSegment(ranges,scan,continous_points,0.04);       

        convertTrailingToPoint(scan, continous_points, corners);
        // convertTrailingToPoint(ranges,scan, continous_points, corners);

        if (findGoodStation(scan, continous_points, info, detect_infos)) {
        // if (findGoodStation(ranges,scan, continous_points, info, detect_infos)) {
            return true;
        }
        return false;
    
    }
    
    //提取点云连续片段
    void extractContinueSegment(std::vector<float>& ranges,ScanType& scan,std::vector<ContinousPoints> &continous_points,double min_dis){
        // auto& ranges=scan->ranges;
        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        int range_size=ranges.size();

        Point point1,point2;
        int point_num=0;    
        for (int i=0;i<range_size-1;i++){
            if (ranges[i] < 3 ){    //提取距离
                double angle_1=angle_min+angle_incre*i;
                double angle_2=angle_min+angle_incre*(i+1);
                point1=Point(ranges[i]*cos(angle_1),ranges[i]*sin(angle_1));
                point2=Point(ranges[i+1]*cos(angle_2),ranges[i+1]*sin(angle_2));
                if ((point1-point2).norm() < min_dis){
                    point_num+=1; 
                }
                else{
                    if (point_num>30){  //所定义连续片段点数不小于10
                        continous_points.emplace_back();
                        continous_points.back().start_index=i-point_num;
                        continous_points.back().end_index=i;
                    }
                    point_num=0;
                }
            }else{
                point_num=0;
            }
        }
    }

    void splitScan(const std::vector<float>& ranges,double angle_increment, std::vector<ContinousPoints> &continous_points) {
        
        int range_size = ranges.size() - 1;
        int start_index = 0;
        int zero_point_num = 0;
        double cos_increment = cos(angle_increment * 2.0);
        double theta_thresh = sin(angle_increment * 2.0) / sin(0.05);//临界值,用于识别断点，默认值0.15
        std::deque<std::pair<int, float>> valid_data;
        bool first_flag = true;
        for (int i = 2; i < range_size; ++i) {
            bool is_trail_point = false;
            if (ranges[i] < max_laser_range && ranges[i] > min_laser_range) {

                if (first_flag) {
                    first_flag = false;
                    valid_data.push_back(std::pair<int, float>(i-2, ranges[i - 2]));
                    valid_data.push_back(std::pair<int, float>(i-1, ranges[i - 1]));
                    valid_data.push_back(std::pair<int, float>(i,ranges[i]));
                } else {
                    valid_data.pop_front();
                    valid_data.push_back(std::pair<int, float>(i, ranges[i]));

                    auto delta_range_1 = valid_data[1].second - valid_data[0].second;
                    auto delta_range_2 = valid_data[2].second - valid_data[1].second;
                    if (delta_range_1 * delta_range_2 >= 0) {
                        double dist_1 = std::sqrt(valid_data[2].second * valid_data[2].second +
                                                  valid_data[0].second * valid_data[0].second -
                                                  2 * valid_data[2].second * valid_data[0].second * cos_increment);
                        double range_thresh_1 = valid_data[1].second * theta_thresh + scan_measure_err;
                        if (dist_1 > range_thresh_1) {
                            is_trail_point = true;
                        }
                    }
                }
                zero_point_num = 0;
            }else {
                zero_point_num++;
                if (zero_point_num > 4)
                    is_trail_point = true;
            }

            if (ranges[i] > max_laser_range || (ranges[i] < min_laser_range && ranges[i]) || is_trail_point) {
                if ((i - start_index) > min_points_size && valid_data.size() == 3 && valid_data[1].first > start_index) {
                    continous_points.emplace_back();
                    continous_points.back().start_index = start_index;
                    // continous_points.back().end_index = i - 1;
                    continous_points.back().end_index = valid_data[1].first;
                }
                // start_index = i + 1;
                if (valid_data.size() < 3)
                    start_index = i + 1;
                else
                    start_index = valid_data[2].first;
            }
        }
        if (continous_points.empty()) {
            continous_points.emplace_back();
            continous_points.back().start_index = start_index;
            continous_points.back().end_index = range_size - 1;
        }
    }

    //重载
    void splitScan(ScanType &scan, std::vector<ContinousPoints> &continous_points,int step) {
        std::vector<float> ranges=scan->ranges;
        // for (int i=0;i<ranges.size();i++ ){
        //     LOG(INFO) << "ranges[" << i  << "]" << ": " << ranges[i];
        // }

        int ranges_size=ranges.size();
        double angle_increment=scan->angle_increment;
        double angle_min=scan->angle_min;
        std::vector<Point> points;
        std::vector<int> IDs;
        Point sumVector_1,sumVector_2;
        // double angle1,angle2;

        for (int i=step;i<ranges_size-step;i++){
            // points.clear();
            for (int j=i-step;j<=i;j++){
                points.emplace_back();
                // LOG(INFO) << "angle_increment: " << angle_increment;
                // LOG(INFO) << "angle_min: " << angle_min;
                double angle=angle_min+angle_increment*(float)j;
                points.back().x=ranges[j]*cos(angle);
                points.back().y=ranges[j]*sin(angle);
                // LOG(INFO) << "x: " << j << "," << points.back().x;
                // LOG(INFO) << "y: " << j<< "," << points.back().y;
            }
            // angle1=calLineAngle(points);
            // Point sumVector=Point(points[1].x-points[0].x,points[1].y-points[0].y);
            // LOG(INFO) << "POINT_1_SIZE: " << points.size();
            sumVector_1=calTrailPoint(points);
            points.clear();

            // LOG(INFO) << "AFTER_POINT_1_SIZE: " << points.size();
            for (int j=i;j<step+i;j++){
                points.emplace_back();
                double angle=angle_min+angle_increment*(float)j;
                points.back().x=ranges[j]*cos(angle);
                points.back().y=ranges[j]*sin(angle);
            }
            // angle2=calLineAngle(points);

            // LOG(INFO) << "POINT_2_SIZE: " << points.size();
            sumVector_2=calTrailPoint(points);
            // LOG(INFO) << "sumVector_1: " << sumVector_1.x << "," << sumVector_1.y;
            // LOG(INFO) << "sumVector_2: " << sumVector_2.x << "," << sumVector_2.y;
            // double theta=acos((sumVector_1*sumVector_2)/(fmod(sumVector_1)*fmod(sumVector_2)))
            sumVector_1.unitlize();
            sumVector_2.unitlize(); 
            // if (fabs(angle1-angle2)< M_PI_2+2*angle_offset && fabs(angle1-angle2)> M_PI_2-2*angle_offset){
            //     IDs.push_back(i);
            // } 
            if (fabs(sumVector_1*sumVector_2)<0.05){
                IDs.push_back(i);
            }

            points.clear();
            // LOG(INFO) << "AFTER_POINT_2_SIZE: " << points.size();

        }
        // LOG(INFO) << "IDs.size: " << IDs.size();
        for (int k=0;k<IDs.size()-1;k++){
            // LOG(INFO) << "IDs: " << k << ",value: " << IDs[k];
            continous_points.emplace_back();
            continous_points.back().start_index=IDs[k];
            continous_points.back().end_index=IDs[k+1];
        }
        IDs.clear();
    }

    double calLineAngle(std::vector<Point>& points){
        int n=points.size();
        double tan_theta;
        double theta;
        double sum_x=0,sum_y=0,sum_xy=0,sum_xx=0;
        for (auto& point:points){
            sum_x=sum_x+point.x;
            sum_y=sum_y+point.y;
            sum_xy=sum_xy+point.x*point.y;
            sum_xx=sum_xx+point.x*point.x;
        }
        tan_theta=(n*sum_xy-sum_x*sum_y)/(n*sum_xx-sum_x*sum_x);
        theta=atan(tan_theta);
        // LineSolver solve;
        // solve::normalizeAngle(theta);
        if (theta > M_PI || theta < -M_PI){
            LOG(INFO) << "The theta is wrong!!!  " << theta;
        }
        return theta;
    }



    //计算散点向量和
    Point calTrailPoint(std::vector<Point>& points){
        Point sumVector=Point(points[1].x-points[0].x,points[1].y-points[0].y);
        for(int i=2;i<points.size();i++){
            Point MiddleVector=Point(points[i].x-points[0].x,points[i].y-points[0].y);
            sumVector=sumVector+MiddleVector;
            // LOG(INFO) << "sumVector: " << sumVector.x << "," << sumVector.y;
        }
        return sumVector;
    }

    //将连续点集合中的点坐标转化为直角坐标
    void convertTrailingToPoint(ScanType &scan, std::vector<ContinousPoints> &continous_points,
                                std::vector<Corner> &corners) {

        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        auto &ranges = scan->ranges;
        for (auto &segment:continous_points) {
            int i = segment.start_index;
            auto range = ranges[i];
            if (range > min_laser_range) {
                double angle = angle_min + angle_incre * (float) i;
                corners.emplace_back();
                corners.back().point = Point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
                corners.back().index = i;
            }
            i = segment.end_index;
            range = ranges[i];
            if (range > min_laser_range) {
                corners.emplace_back();
                double angle = angle_min + angle_incre * (float) i;
                corners.back().point = Point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
                corners.back().index = i;
            }
        }
    }
     
//     //将单点转化为直角坐标
    void convertSinglePoint(const std::vector<float>& ranges,ScanType &scan,int index,Corner& corner){
        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        auto range = ranges[index];
            if (range > min_laser_range) {
                double angle = angle_min + angle_incre * (float) index;
                corner.point = Point(cos(angle) * range, sin(angle) * range);
                corner.index = index;
            } else {
                LOG(INFO) << "range is wrong:" << range;
            }
        }
    

    void buildPoints(ScanType &scan, const ContinousPoints &continous, std::vector<Point> &points) {
        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        auto &ranges = scan->ranges;
        for (int i = continous.start_index; i <= continous.end_index; ++i) {
            auto &range = ranges[i];
            if (range > min_laser_range) {
                double angle = angle_min + angle_incre * (float) i;
                points.emplace_back(Point(cos(angle) * ranges[i], sin(angle) * ranges[i]));
            }
        }
    }

    bool findGoodStation(ScanType &scan, std::vector<ContinousPoints> &continous_points,
                             Goodinfo &info, std::vector<GoodDetectInfo> &detect_infos) {
        // auto &check_length = info.baseline;
        // auto angle_min = scan->angle_min;
        // auto angle_incre = scan->angle_increment;
        auto &ranges = scan->ranges;
        // for (int i=0;i<continous_points.size()-1;i++){
        //     Corner first_corner, second_corner;

        //     convertSinglePoint(ranges,scan,continous_points[i].start_index,first_corner);
        //     convertSinglePoint(ranges,scan,continous_points[i].end_index,second_corner);
        //     // convertSinglePoint(ranges,scan,continous_points[i+1].start_index,first_corner_2);
        //     // convertSinglePoint(ranges,scan,continous_points[i+1].end_index,second_corner_2);
        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        // auto &ranges = scan->ranges;
        for (auto &segment:continous_points) {
            Corner first_corner, second_corner;
            int i = segment.start_index;
            auto range = ranges[i];
            if (range > min_laser_range) {
                double angle = angle_min + angle_incre * (float) i;
                first_corner.point = Point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
                first_corner.index = i;
            } else {
                LOG(INFO) << "range is wrong:" << range;
            }
            i = segment.end_index;
            range = ranges[i];
            if (range > min_laser_range) {
                double angle = angle_min + angle_incre * (float) i;
                second_corner.point = Point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
                second_corner.index = i;
            } else {
                LOG(INFO) << "range is wrong:" << range;
            }
            // LOG(INFO) << "start_index: " << first_corner.index <<  ",(X1,Y1): " << "(" << first_corner.point.x << "," << first_corner.point.y << ")";
            // LOG(INFO) << "end_index: "<< second_corner.index << ",(X2,Y2): " << "(" << second_corner.point.x << "," << second_corner.point.y << ")";
            // std::cout << std::endl;
            double target_num_1=first_corner.point.y;
            double target_num_2=second_corner.point.y;

            if (inRange((first_corner.point - second_corner.point).norm(), info.baseline, 2 * max_dist_err)){
                    // LOG(INFO) << "index: " << first_corner.index <<  ",(X1,Y1): " << "(" << first_corner.point.x << "," << first_corner.point.y << ")";
                    // LOG(INFO) << "index: " << second_corner.index << ",(X2,Y2): " << "(" << second_corner.point.x << "," << second_corner.point.y << ")";
                    // LOG(INFO) << "target_num_1: " << target_num_1 << ", target_num_2: " << target_num_2;

                    // std::cout << std::endl;
                if (getMaxHeight(scan,first_corner.index,second_corner.index) <2*max_dist_err){
                    LOG(INFO) << "index1: " << first_corner.index <<  ",(X1,Y1): " << "(" << first_corner.point.x << "," << first_corner.point.y << ")";
                    LOG(INFO) << "index2: " << second_corner.index << ",(X2,Y2): " << "(" << second_corner.point.x << "," << second_corner.point.y << ")";
                    std::cout << std::endl;
                    if (target_num_1*target_num_2 <0){
                        calError(first_corner,second_corner);
                        return 1;
                    }
                }
            }
                    
        // return !detect_infos.empty();
        }
    }


    void filter(ScanType& scan,double max_distance,double min_thresh ){
        ranges = scan->ranges;
        std::vector<int> indexes;
        const int step = 2;
        double cos_increment = cos(scan->angle_increment * (double)step);
        double theta_thresh=sin((double)scan->angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
        int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance) {
                continue;
            }
            double dist_direction = (ranges[i + step] - ranges[i - step]);            

            for (int k = -step; k < step; ++k) {
                double tmp_direction = (ranges[i + k + 1] - ranges[i + k]);
                if (dist_direction * tmp_direction <= 0) {
                    continue;
                }
            }
            double dist_1 = std::sqrt(ranges[i] * ranges[i] + ranges[i - step] * ranges[i - step] -
                                    2 * ranges[i] * ranges[i - step] * cos_increment);
            double dist_2 = std::sqrt(ranges[i] * ranges[i] + ranges[i + step] * ranges[i + step] -
                                    2 * ranges[i] * ranges[i + step] * cos_increment);
            // double dist_1=ranges[i]*cos((double)scan->angle_increment * (double)step);
            // double dist_2=ranges[i+step]*cos((double)scan->angle_increment * (double)step);

            double range_thresh_1 = ranges[i] * theta_thresh + min_thresh;
            double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh;
            // double range_thresh_1 = ranges[i-step] + min_thresh;
            // double range_thresh_2 = ranges[i] + min_thresh;
            // std::cout << "dist_1: " << dist_1 << ", range_thresh_1: "<< range_thresh_1 << std::endl;
            // std::cout << "dist_2: " << dist_2 << ", range_thresh_2: "<< range_thresh_2 << std::endl;
            // std::cout << std::endl;
            if(dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
                for (int j = -step; j <= step; ++j) {
                    indexes.push_back(i + j);
                }
            }

        }
        for (auto &index:indexes) {
            ranges[index] = 100;
        }

   

}



    //计算偏差
    double calError(Corner& pose1,Corner& pose2){

        Point mid_pose;  //中心点   TODO:此处需处理，点云滤波，投影等操作
        mid_pose=(pose1.point+pose2.point).scalarMultiply(0.5);
        // std::cout << "mid_pose: " << "(" << mid_pose.x << "," << mid_pose.y << "," << 0 << ")"  << std::endl;
        Eigen::Vector3d p(mid_pose.x,mid_pose.y,0); //避障雷达坐标系下的点p
        Eigen::Vector3d eulerAngles(0,-0.523,0);    //欧拉角，弧度制
        Eigen::Quaterniond q;        //四元数
        Eigen::Vector3d t(0.466,0,0.6255);  //平移量,由车体中心坐标系原点指向避障雷达中心坐标系原点

        //欧拉角转化为四元数
        Eigen::AngleAxisd roll(eulerAngles[0],Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(eulerAngles[1],Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(eulerAngles[2],Eigen::Vector3d::UnitZ());
        q=yaw*pitch*roll;
        // q.normalize();  //正则化

        Eigen::Isometry3d T1w(q); //从车体中心到避障雷达的欧氏变换
        T1w.pretranslate(t);

        Eigen::Vector3d res=T1w.inverse()*p;    //TODO
        // std::cout << "T1w" << T1w.matrix() << std::endl;
        // std::cout << std::endl << "res: " << "(" << res.transpose() << ")" << std::endl;
        return res[1];
    }
    
    //计算两点之间

    double getMaxHeight(const ScanType &scan, int start_index, int end_index) {
        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        auto &ranges = scan->ranges;
        double start_angle = angle_min + angle_incre * (float) start_index;
        Point start_point = Point(cos(start_angle) * ranges[start_index], sin(start_angle) * ranges[start_index]);

        double end_angle = angle_min + angle_incre * (float) end_index;
        Point end_point = Point(cos(end_angle) * ranges[end_index], sin(end_angle) * ranges[end_index]);

        Point unit_vecor = (start_point - end_point);
        unit_vecor.unitlize();
        double max_height = 0.0;
        for (int i = start_index + 1; i < end_index; ++i) {
            if (ranges[i]) {
                double angle = angle_min + angle_incre * (float) i;
                Point point = Point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
                auto height = (point - end_point).height(unit_vecor);
                if (max_height < height) {
                    max_height = height;
                }
            } 
        }
        return max_height;
    }

    void getTriangeEdgeLength(const Point &point, const Point &first, const Point &second,
            const Goodinfo &info,double &dist_1, double &dist_2) {
        dist_1 = (point - first).norm();
        dist_2 = (point - second).norm();
    }

    double getTrapeziaIncludedAngle(const Point &first_corner, const Point &first_top, const Point &second_corner,
                                    const Point &second_top) {
        auto delta_point_1 = first_corner - first_top;
        auto delta_point_2 = second_corner - second_top;
        double delta_dist_1 = delta_point_1.norm();
        double delta_dist_2 = delta_point_2.norm();
        if (delta_dist_1 == 0) {
            return 0;
        }
        if (delta_dist_2 == 0) {
            return 0;
        }
        double cos_angle = (delta_point_1 * delta_point_2) / (delta_dist_1 * delta_dist_2);
        return acos(cos_angle);
    }

    //计算离散点拟合后的的直线倾斜角
    double computeLinePara(const ScanType &scan, const Corner &start, const Corner &end, LineSolver::LinePara &line_para) {
        std::vector<Point> top_line_points;
        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        auto &ranges = scan->ranges;
        for (int i = start.index; i < end.index; ++i) {
            auto &range = ranges[i];
            double angle = angle_min + angle_incre * (float) i;
            top_line_points.emplace_back(Point(cos(angle) * range, sin(angle) * range));
        }
        LineSolver line_solver;
        return line_solver.solve(top_line_points, line_para);

    }


    bool inRange(const double curr_dist, const double check_length, const double err_thresh) {
        double delta_dist = fabs(curr_dist - check_length);
        return delta_dist < err_thresh;
    }


    bool extractCorners(std::vector<Point> &points, std::vector<Corner> &corners) {
        Corner first_corner;
        first_corner.point = points[0];
        first_corner.index = 0;
        corners.push_back(first_corner);
        first_corner.point = points.back();
        first_corner.index = points.size() - 1;
        corners.push_back(first_corner);

        int end_index = points.size() - 1;
        std::stack<ContinousPoints> segments;
        ContinousPoints first_segment;
        first_segment.start_index = 0;
        first_segment.end_index = end_index;
        segments.push(first_segment);
        while (!segments.empty()) {
            auto segment = segments.top();
            segments.pop();
            int corner_index;
            auto height = getMaxHeight(points, segment.start_index, segment.end_index, corner_index);
            if (height > min_corner_height) {
                ContinousPoints first, second;
                first.start_index = segment.start_index;
                first.end_index = corner_index;
                second.start_index = corner_index;
                second.end_index = segment.end_index;
                segments.push(first);
                segments.push(second);
                corners.emplace_back();
                corners.back().point = points[corner_index];
                corners.back().index = corner_index;
            }
        }
        return !corners.empty();
    }

    double getMaxHeight(const std::vector<Point> &points, int min_index, int max_index, int &corner_index) {
        auto &first_point = points[min_index];
        Point unit_vecor = (points[max_index] - first_point);
        unit_vecor.unitlize();
        double max_height = 0;
        for (int i = min_index + 1; i < max_index; ++i) {
            auto height = (points[i] - first_point).height(unit_vecor);
            if (max_height < height) {
                max_height = height;
                corner_index = i;
            }
        }
        return max_height;
    }

private:
    const double max_laser_range = 3.0;
    const double min_laser_range = 0.3;
    const double break_point_dist = 0.15;
    const int min_points_size = 20;
    const double min_corner_height = 0.05;
    const double max_dist_err = 0.05;
    const double scan_measure_err = 0.03;
    double angle_offset=0.0872;

    std::vector<float> ranges;
    
};

}


#endif //PROJECT_CHARGING_STATION_DETECTOR_HPP1
