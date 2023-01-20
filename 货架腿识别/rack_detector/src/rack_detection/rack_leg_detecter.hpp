//
// Created by getup on 18-12-5.
//

#ifndef PROJECT_RACK_LEG_DETECTER_HPP
#define PROJECT_RACK_LEG_DETECTER_HPP

//#include <visualization_msgs/Marker.h>
#include "scan_processor/scan_processor.hpp"

namespace rack_detection {
template <class LaserScan>
class RackLegDetecter {
 public:
    RackLegDetecter() = default;

    RackLegDetecter(const RackLegDetecter &rhs) = delete;

    RackLegDetecter &operator=(const RackLegDetecter &rhs) = delete;

 private:
    static constexpr int leg_min_points_ = 4;
    ScanProcessor<LaserScan> scan_processor_;
    std::vector<Eigen::Array2i> regions_;

 public:
    void Set(std::shared_ptr<LaserScan> scan_ptr) { GetContinuousRegions(scan_ptr); }

    void GetRackLegs(const float leg_max_radius, std::vector<Eigen::Vector2f> &rack_legs) {
        const auto &scan = scan_processor_.GetScan();
        rack_legs.clear();
        for (const auto &region : regions_) {
            std::vector<Eigen::Vector2f> points;
            Eigen::Vector2f mean = Eigen::Vector2f::Zero();
            for (int i = region.x(); i <= region.y(); i++) {
                const double angle = scan.angle_min + scan.angle_increment * static_cast<double>(i);
                points.emplace_back(scan.ranges[i] * std::cos(angle), scan.ranges[i] * std::sin(angle));
                mean += points.back();
            }
            mean *= 1.f / static_cast<float>(points.size());
            bool is_near = false;
            for (const auto &leg : rack_legs) {
                if ((leg - mean).norm() < leg_max_radius) {
                    is_near = true;
                    break;
                }
            }
            if (!is_near) {
                float radius = 0.f;
                for (const auto &point : points) {
                    const float dist = (point - mean).norm();
                    if (dist > radius) radius = dist;
                }
                if (radius <= leg_max_radius) {
                    rack_legs.push_back(mean);
                }
            }
        }
    }

    // template <class RosLaserScan>
    // void DebugDisplay(RosLaserScan &scan) const {
    //     const auto &filted_scan = scan_processor_.GetScan();
    //     scan.angle_increment = filted_scan.angle_increment;
    //     scan.angle_max = filted_scan.angle_max;
    //     scan.angle_min = filted_scan.angle_min;
    //     scan.intensities = filted_scan.intensities;
    //     scan.range_max = filted_scan.range_max;
    //     scan.range_min = filted_scan.range_min;
    //     scan.ranges = filted_scan.ranges;
    // }

 private:
    void GetContinuousRegions(std::shared_ptr<LaserScan> scan_ptr) {
        scan_processor_.SetScan(*scan_ptr);
        scan_processor_.FiltInvalidRange();
        scan_processor_.FiltInvalidAngle(-3.14159, 3.1415);
        scan_processor_.GetPointRegions(regions_, leg_min_points_);
    }
};
}  // namespace rack_detection

#endif  // PROJECT_RACK_LEG_DETECTER_HPP
