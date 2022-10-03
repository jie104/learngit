//
// Created by lfc on 19-2-14.
//

#ifndef PROJECT_CHARGING_STATION_DETECTOR_HPP
#define PROJECT_CHARGING_STATION_DETECTOR_HPP

#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <stack>
#include "line_solver.hpp"

namespace detector {
struct ChargingStationInfo {
    double topline = 0.216;
    double baseline = 0.496;
    double waist = 0.1612;
    double waist_to_base = 0.3648;
    double included_angle = 2.0943;
};

struct ChargingStationDetectInfo {
    double direction = 0;
    Point center_point;
    Point first_corner;
    Point second_corner;
    Point first_top;
    Point second_top;
};

template <class ScanType>
class ChargingStationDetector {
public:
    struct ContinousPoints {
        int start_index;
        int end_index;
    };


    struct Corner {
        Point point;
        int index;
    };

    ChargingStationDetector() {

    }

    bool findChargingStation(ScanType &scan, ChargingStationInfo &info,std::vector<detector::ChargingStationDetectInfo> &detect_infos) {
        std::vector<ContinousPoints> continous_points;
        std::vector<Corner> corners;
        splitScan(scan->ranges, scan->angle_increment, continous_points);
        convertTrailingToPoint(scan, continous_points, corners);
        if (findChargingStation(scan, continous_points, info, detect_infos)) {
            return true;
        }
        return false;
    }

    void splitScan(std::vector<float> &ranges, double angle_increment, std::vector<ContinousPoints> &continous_points) {
        int range_size = ranges.size() - 1;
        int start_index = 0;
        double cos_increment = cos(angle_increment * 2.0);
        double theta_thresh = sin(angle_increment * 2.0) / sin(0.15);//临界值,用于识别断点
        for (int i = 1; i < range_size; ++i) {
            bool is_trail_point = false;
            if (ranges[i] < max_laser_range && ranges[i] > min_laser_range) {
                auto delta_range_1 = ranges[i] - ranges[i - 1];
                auto delta_range_2 = ranges[i + 1] - ranges[i];
                if (delta_range_1 * delta_range_2 > 0) {
                    double dist_1 = std::sqrt(ranges[i + 1] * ranges[i + 1] + ranges[i - 1] * ranges[i - 1] -
                                              2 * ranges[i + 1] * ranges[i - 1] * cos_increment);
                    double range_thresh_1 = ranges[i] * theta_thresh + scan_measure_err;
                    if (dist_1 > range_thresh_1) {
                        is_trail_point = true;
                    }
                }
            }

            if (ranges[i] > max_laser_range || ranges[i] < min_laser_range || is_trail_point) {
                if ((i - start_index) > min_points_size) {
                    continous_points.emplace_back();
                    continous_points.back().start_index = start_index;
                    continous_points.back().end_index = i - 1;
                }
                start_index = i + 1;
            }

        }
        if (continous_points.empty()) {
            continous_points.emplace_back();
            continous_points.back().start_index = start_index;
            continous_points.back().end_index = range_size - 1;
        }
    }

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

    bool findChargingStation(ScanType &scan, std::vector<ContinousPoints> &continous_points,
                             ChargingStationInfo &info, std::vector<ChargingStationDetectInfo> &detect_infos) {
        auto &check_length = info.baseline;
        double projection_length = (info.baseline - info.topline) / 2.0;
        double check_height = info.waist * info.waist - projection_length * projection_length;
        double min_topline_length = info.topline - max_dist_err;

        if (check_height >= 0) {
            check_height = sqrt(check_height);
        } else {
            LOG(INFO) << "check length is wrong!" << check_height;
            check_height = 0.08;
        }

        auto angle_min = scan->angle_min;
        auto angle_incre = scan->angle_increment;
        auto &ranges = scan->ranges;
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
            if (inRange((first_corner.point - second_corner.point).norm(), info.baseline, 2 * max_dist_err)) {
                Corner waist_1, waist_2;
                double initial_err_1 = 2 * max_dist_err;
                double initial_err_2 = 2 * max_dist_err;
                for (int j = first_corner.index; j < second_corner.index; ++j) {
                    double angle = angle_min + angle_incre * (float) j;
                    auto point = Point(cos(angle) * ranges[j], sin(angle) * ranges[j]);
                    double dist_1, dist_2, dist_2_1, dist_2_2;
                    getTriangeEdgeLength(point, first_corner.point, second_corner.point, info, dist_1, dist_2);
                    double delta_err_1_1 = fabs(dist_1 - info.waist);
                    double delta_err_1_2 = fabs(dist_2 - info.waist_to_base);
                    double delta_err_1 = std::hypot(delta_err_1_1, delta_err_1_2);

                    double delta_err_2_1 = fabs(dist_1 - info.waist_to_base);
                    double delta_err_2_2 = fabs(dist_2 - info.waist);
                    double delta_err_2 = std::hypot(delta_err_2_1, delta_err_2_2);
                    if (delta_err_1 < initial_err_1) {
                        initial_err_1 = delta_err_1;
                        waist_1.index = j;
                        waist_1.point = point;
                    }
                    if (delta_err_2 < initial_err_2) {
                        initial_err_2 = delta_err_2;
                        waist_2.index = j;
                        waist_2.point = point;
                    }
                }
                if (initial_err_1 < 2 * max_dist_err && initial_err_2 < 2 * max_dist_err) {

                    double included_angle = getTrapeziaIncludedAngle(first_corner.point, waist_1.point,
                                                                     second_corner.point, waist_2.point);
                    if (inRange((waist_1.point - waist_2.point).norm(), info.topline, max_dist_err)) {
                        if (inRange(included_angle, info.included_angle, 0.25)) {
                            auto height_1 = getMaxHeight(scan, first_corner.index, waist_1.index);
                            auto height_2 = getMaxHeight(scan, waist_1.index, waist_2.index);
                            auto height_3 = getMaxHeight(scan, waist_2.index, second_corner.index);
//                            LOG(INFO) << "height_1:" << height_1 << "," << height_2 << "," << height_3;
                            if (height_1 < 2 * max_dist_err && height_2 < 2 * max_dist_err &&
                                height_3 < 2 * max_dist_err) {
                                detect_infos.emplace_back();
                                detect_infos.back().first_corner = first_corner.point;
                                detect_infos.back().second_corner = second_corner.point;
                                detect_infos.back().first_top = waist_1.point;
                                detect_infos.back().second_top = waist_2.point;
                                computeChargingCenterPose(scan, first_corner, waist_1, second_corner, waist_2,
                                                          detect_infos.back());
                                Corner center_corner;
                                center_corner.point = detect_infos.back().center_point;
                            }
                        }
//                    return true;
                    }
                }
            }

        }
        return !detect_infos.empty();
    }

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
            double angle = angle_min + angle_incre * (float) i;
            Point point = Point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
            auto height = (point - end_point).height(unit_vecor);
            if (max_height < height) {
                max_height = height;
            }
        }
        return max_height;
    }

    void getTriangeEdgeLength(const Point &point, const Point &first, const Point &second,
            const ChargingStationInfo &info,double &dist_1, double &dist_2) {
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

    void computeChargingCenterPose(const ScanType &scan, const Corner &first_corner, const Corner &first_top,
                                   const Corner &second_corner, const Corner &second_top,
                                   ChargingStationDetectInfo &detect_info) {
        LineSolver::LinePara line_para_1, line_para_2, line_para_3;
        computeLinePara(scan, first_top, second_top, line_para_1);

        Point unit_vector(cos(line_para_1.angle), sin(line_para_1.angle));
        Point line_origin_point;
        if (fabs(line_para_1.A) > fabs(line_para_1.B)) {
            line_origin_point.y = 0;
            line_origin_point.x = -line_para_1.C / line_para_1.A;
        } else {
            line_origin_point.x = 0;
            line_origin_point.y = -line_para_1.C / line_para_1.B;
        }
        auto first_corner_map =
                unit_vector.scalarMultiply(unit_vector * (first_corner.point - line_origin_point)) + line_origin_point;
        auto second_corner_map =
                unit_vector.scalarMultiply(unit_vector * (second_corner.point - line_origin_point)) + line_origin_point;
        auto first_top_map =
                unit_vector.scalarMultiply(unit_vector * (first_top.point - line_origin_point)) + line_origin_point;
        auto second_top_map =
                unit_vector.scalarMultiply(unit_vector * (second_top.point - line_origin_point)) + line_origin_point;
        auto center = (first_corner_map + second_corner_map + first_top_map + second_top_map).scalarMultiply(0.25);
        double charging_direction = line_para_1.angle + M_PI_2;
        computeLinePara(scan, first_corner, first_top, line_para_2);
        computeLinePara(scan, second_top, second_corner, line_para_3);
        auto unit_vector_2 = Point(cos(line_para_2.angle), sin(line_para_2.angle));
        auto unit_vector_3 = Point(cos(line_para_3.angle), sin(line_para_3.angle));
        Point sum_point;
        if (unit_vector_2 * unit_vector_3 < 0) {
            sum_point = unit_vector_2 + unit_vector_3;
        } else {
            sum_point = unit_vector_2 - unit_vector_3;
        }
        sum_point.unitlize();
        double vector_angle = atan2(sum_point.y, sum_point.x);
        auto direct_bk = Point(cos(charging_direction), sin(charging_direction));

        if (sum_point * direct_bk < 0) {
            sum_point = direct_bk - sum_point;

        } else {
            sum_point = direct_bk + sum_point;//使用向量法计算方向的均值
        }

        double sum_mean_angle = atan2(sum_point.y, sum_point.x);
        if (sum_point * center > 0.0) {
            sum_mean_angle += M_PI;
        }

        if (Point(cos(charging_direction), sin(charging_direction))*center > 0.0){
            charging_direction += M_PI;
            LineSolver::normalizeAngle(charging_direction);
        }

        detect_info.center_point = center;
        detect_info.direction = sum_mean_angle;

//        center_point = center;
        LOG(INFO) << "angle:" << charging_direction * 180 / M_PI << ","
                  << (sum_mean_angle) * 180.0 / M_PI << "," << (vector_angle + M_PI) * 180.0 / M_PI
                  << "," << center.x << ","
                  << center.y;
    }

    void computeLinePara(const ScanType &scan, const Corner &start, const Corner &end, LineSolver::LinePara &line_para) {
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
        line_solver.solve(top_line_points, line_para);
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
    const double max_dist_err = 0.03;
    const double scan_measure_err = 0.03;
};

}


#endif //PROJECT_CHARGING_STATION_DETECTOR_HPP
