//
// Created by lfc on 18-12-15.
//

#ifndef SROS_SCAN_LASER_FILTER_ALG_HPP
#define SROS_SCAN_LASER_FILTER_ALG_HPP
namespace laser{

/**
 * @brief 过滤range小于max_distance的拖尾点
 * 
 * @tparam LaserScan_ptr 
 * @param scan 
 * @param max_distance 
 */
template <class LaserScan_ptr>
void filterScanTraildPoint(LaserScan_ptr scan,double max_distance) {
    auto& ranges = scan->ranges;
    std::vector<int> indexes;
    double min_thresh = 0.02;
    const int step = 2;
    double cos_increment = cos(scan->angle_increment * (double)step);
    double theta_thresh=sin((double)scan->angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
    int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
    for (int i = step; i < scan_size; i++) {
        if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance) {
            continue;
        }
        double dist_direction = (ranges[i + step] - ranges[i - step]);  //?? 没用
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
        double range_thresh_1 = ranges[i] * theta_thresh + min_thresh;
        double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh;
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

/**
 * @brief 将range小于max_distance的低强度point的range置0
 * 
 * @tparam LaserScan_ptr 
 * @param scan 
 * @param max_distance 
 */
template <class LaserScan_ptr>
void filterLowIntenPoints(LaserScan_ptr scan,double max_distance){
    int scan_size = scan->ranges.size();

    double tail_center_angle_min = -1.5707;
    double tail_center_angle_max = 1.5707;

    double angle = scan->angle_min;
    double step = scan->angle_increment;

    int first_tail_center = (int) round((tail_center_angle_min - angle) / step);
    int second_tail_center = (int) round((tail_center_angle_max - angle) / step);

    int delta_thresh = (int) round(0.51 / scan->angle_increment);

    for (int i = 0; i < scan_size; ++i) {
        if ((i - first_tail_center) < delta_thresh || (second_tail_center - i) < delta_thresh) {
            if (scan->ranges[i] < max_distance) {
                if (scan->intensities[i] < 150) {
                    scan->ranges[i] = 0;
                }
            }
        }
    }
}

template <class LaserScan_ptr>
void filterScanPoint(LaserScan_ptr scan) {
    auto& ranges = scan->ranges;
    double theta_thresh=sin((double)scan->angle_increment)/sin(0.170);//临界值,用于识别断点
    double min_thresh = 0.03f;
    int scan_size = ranges.size() - 1;
    std::vector<int> indexes;
    for (int i = 1; i < scan_size; i++) {
        if (ranges[i] == 100 || ranges[i] == 0) {
            continue;
        }
        float dist_1 = fabs(ranges[i] - ranges[i - 1]);
        float dist_2 = fabs(ranges[i + 1] - ranges[i]);
        float range_thresh_1 = ranges[i - 1] * theta_thresh + min_thresh;
        float range_thresh_2 = ranges[i + 1] * theta_thresh + min_thresh;
        if (dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
            indexes.push_back(i);
        }
    }
    for (auto &index:indexes) {
        ranges[index] = 100;
    }
}

}
#endif //SROS_SCAN_LASER_FILTER_ALG_HPP
