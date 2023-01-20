#include <vector>
#include <memory>
#include "sensor_msgs/LaserScan.h"


namespace KM {
    struct ScanPoint {
        struct ScanRange {
            float range;
            float angle;
        };

        std::vector<ScanRange> points;
        ScanRange min_range;
        ScanRange max_range;
    };

// template void KM::getClusterFromScan<sensor_msgs::LaserScan::ConstPtr>(sensor_msgs::LaserScan::ConstPtr&,std::vector<std::shared_ptr<KM::ScanPoint>>&,
//         const int,const double);

// template bool KM::inThresh<float>(const float&,const float&,const double&,const double&,const double&);

// template const KM::ScanPoint::ScanRange KM::creatScanRange<float>(const float&,const int&,const double&,const double&);
    template <class RangeType>
    const ScanPoint::ScanRange creatScanRange(const RangeType& range,const int &index,const double& angle_min,const double& angle_increment){
        ScanPoint::ScanRange point;
        point.angle = index * angle_increment + angle_min;
        point.range = range;
        return point;
    }

    template <class RangeType>  //正常阈值内的点，不在阈值内的点过滤
    bool inThresh(const RangeType& range_1,const RangeType& range_2,const double &cos_increment,const double &theta_thresh,const double& min_thresh){
        double dist_1 = std::sqrt(range_1 * range_1 + range_2 * range_2 -
                                2 *range_1 * range_2 * cos_increment);
        double range_thresh_1 = range_1 * theta_thresh + min_thresh;
        if(dist_1>range_thresh_1){
            return false;
        }
        return true;
    }

    template <class LaserScan_ptr>
    void getClusterFromScan(const LaserScan_ptr& scan,std::vector<std::shared_ptr<ScanPoint>>& clusters,const int min_cluster_size = 3,
            const double min_thresh = 0.02) {
        const int step = 2;
        double cos_increment = cos(scan->angle_increment);
        double theta_thresh = sin((double)scan->angle_increment) / sin(0.17);  //临界值,用于识别断点
        double cos_increment_2 = cos(scan->angle_increment * step);
        double theta_thresh_2 = sin((double)scan->angle_increment * step) / sin(0.17);  //临界值,用于识别断点

        auto& ranges = scan->ranges;
        auto& angle_increment = scan->angle_increment;
        auto& angle_min = scan->angle_min;
        int scan_size = ranges.size() - step;
        clusters.emplace_back(new ScanPoint);
        clusters.back()->min_range.range = ranges[0];
        clusters.back()->min_range.angle = angle_min;
        clusters.back()->points.push_back(clusters.back()->min_range);
        for (int i = step - 1; i <= scan_size; ++i) {
            auto& last_cluster = clusters.back();
            if (inThresh(ranges[i - 1], ranges[i], cos_increment, theta_thresh, min_thresh)) {
                clusters.back()->points.push_back(creatScanRange(ranges[i],i,angle_min,angle_increment));
            } else if (inThresh(ranges[i - 1], ranges[i + 1], cos_increment_2, theta_thresh_2, min_thresh)) {
                i++;
                clusters.back()->points.push_back(creatScanRange(ranges[i],i,angle_min,angle_increment));
            }else{
                clusters.emplace_back(new ScanPoint);
                clusters.back()->min_range = creatScanRange(ranges[i], i, angle_min, angle_increment);
                clusters.back()->points.push_back(clusters.back()->min_range);
            }
        }
        if(inThresh(clusters.back()->points.back().range,ranges.back(),cos_increment,theta_thresh,min_thresh)){
            clusters.back()->points.push_back(
                creatScanRange(ranges.back(), ranges.size() - 1, angle_min, angle_increment));
        }
        std::vector<std::shared_ptr<ScanPoint>> clusters_bk;
        clusters_bk.swap(clusters);
        clusters.reserve(clusters_bk.size());
        for(auto& cluster:clusters_bk){
            if(cluster->points.size()>=min_cluster_size){
                clusters.push_back(cluster);
            }
        }
    }

// template const KM::ScanPoint::ScanRange KM::creatScanRange<float>(const float&,const int&,const double&,const double&);


}

