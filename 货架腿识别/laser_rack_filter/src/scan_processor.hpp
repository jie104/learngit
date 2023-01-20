//
// Created by getup on 18-6-28.
//

#pragma once

#include <Eigen/Dense>
#include <numeric>

namespace scan {
template <class LaserScanMsg>
class ScanProcessor {
 public:
  ScanProcessor() = default;

  ScanProcessor(const ScanProcessor &rhs) = delete;

  ScanProcessor &operator=(const ScanProcessor &rhs) = delete;

 private:
    LaserScanMsg scan_;
  static constexpr double tailed_angle_threshold_ = 0.17;     //判断拖尾角度阈值(rad)
  static constexpr double sigma_r_ = 0.01;   //距离sigma值;
  double cos_theta_ = 1.0;
  double k_ = 0.0;

 public:
  void SetScan(const LaserScanMsg &scan) {
    scan_ = scan;
    cos_theta_ = std::cos(scan_->angle_increment);
    k_ = std::sin(scan_->angle_increment) / std::sin(tailed_angle_threshold_);
  }

  const LaserScanMsg &GetScan() const {
    return scan_;
  }

  //距离滤波
  void FiltInvalidRange(const double range_min, const double range_max) {
    for (auto &range : scan_->ranges) {
      if (!IsRangeValid(range, range_min, range_max))
        range = 0.f;
    }
  }

  //距离滤波
  void FiltInvalidRange() {
    FiltInvalidRange(scan_->range_min, scan_->range_max);
  }

  //角度滤波
  void FiltInvalidAngle(const double angle_min, const double angle_max) {
    double angle = scan_->angle_min;
    auto iter = scan_->ranges.begin();
    while (iter != scan_->ranges.end()) {
      if (!IsAngleValid(angle, angle_min, angle_max)) {
        *iter = 0.f;
      }
      angle += scan_->angle_increment;
      ++iter;
    }
  }

  //角度滤波
  void FiltInvalidAngle() {
    FiltInvalidAngle(scan_->angle_min, scan_->angle_max);
  }

  //距离中值滤波
  void RangeMedianFilt(const unsigned int half = 5) {
    std::vector<float> new_ranges;
    for (unsigned int pos = 0; pos < half; pos++) {
      if (IsRangeValid(scan_->ranges[pos], scan_->range_min, scan_->range_max))
        new_ranges.push_back(scan_->ranges[pos]);
      else
        new_ranges.push_back(0.f);
    }
    for (unsigned int i = half; i < scan_->ranges.size() - half; i++) {
      std::vector<float> sample_ranges;
      for (unsigned int pos = i - half; pos <= i + half; pos++) {
        if (IsRangeValid(scan_->ranges[pos], scan_->range_min, scan_->range_max))
          sample_ranges.push_back(scan_->ranges[pos]);
      }
      if (sample_ranges.empty())
        new_ranges.push_back(0.f);
      else {
        std::nth_element(sample_ranges.begin(),
                         sample_ranges.begin() + sample_ranges.size() / 2,
                         sample_ranges.end());
        new_ranges.push_back(sample_ranges[sample_ranges.size() / 2]);
      }
    }
    for (unsigned int pos = scan_->ranges.size() - half; pos < scan_->ranges.size(); pos++) {
      if (IsRangeValid(scan_->ranges[pos], scan_->range_min, scan_->range_max))
        new_ranges.push_back(scan_->ranges[pos]);
      else
        new_ranges.push_back(0.f);
    }
    scan_->ranges = new_ranges;
  }

  //区域滤波
  void FiltInvalidRegion(const Eigen::Array2i &region) {
    const unsigned int size = scan_->ranges.size();
    if (region.y() < size) {
      for (int i = region.x(); i <= region.y(); i++) {
        scan_->ranges[i] = 0.f;
      }
    }
    else if (region.x() < size) {
      for (int i = region.x(); i < size; i++) {
        scan_->ranges[i] = 0.f;
      }
      for (int i = 0; i <= region.y() - size; i++) {
        scan_->ranges[i] = 0.f;
      }
    }
    else {
      for (int i = region.x() - size; i <= region.y() - size; i++) {
        scan_->ranges[i] = 0.f;
      }
    }
  }

  //降采样
  void DownSample(const unsigned int size = 3600) {
    if (size >= scan_->ranges.size())
      return;
    const float
        new_angle_increment = (scan_->angle_max - scan_->angle_min + scan_->angle_increment) / static_cast<float>(size);
    std::vector<float> new_ranges;
    std::vector<float> new_intensities;
    float angle = scan_->angle_min;
    float thresh = scan_->angle_min;
    unsigned int pos = 0;
    for (unsigned int i = 0; i < size; i++) {
      thresh += new_angle_increment;
      std::vector<float> sample_ranges;
      std::vector<float> sample_intensities;
      while (angle < thresh) {
        if (IsRangeValid(scan_->ranges[pos], scan_->range_min, scan_->range_max))
          sample_ranges.push_back(scan_->ranges[pos]);
        sample_intensities.push_back(scan_->intensities[pos]);
        angle += scan_->angle_increment;
        pos++;
      }
      if (sample_ranges.empty())
        new_ranges.push_back(0.f);
      else {
        std::nth_element(sample_ranges.begin(),
                         sample_ranges.begin() + sample_ranges.size() / 2,
                         sample_ranges.end());
        new_ranges.push_back(sample_ranges[sample_ranges.size() / 2]);
      }
      new_intensities.push_back(std::accumulate(sample_intensities.cbegin(), sample_intensities.cend(), 0.f)
                                    / static_cast<float>(sample_intensities.size()));
    }
    scan_->angle_increment = new_angle_increment;
    scan_->ranges = new_ranges;
    scan_->intensities = new_intensities;
  }

  //获得连续段对应的序号
  void GetContinuousRegions(std::vector<Eigen::Array2i> &regions) {
    regions.clear();
    unsigned int front_index = 0;
    for (unsigned int i = 1; i < scan_->ranges.size(); i++) {
      if (IsBreakIndex(i)) {
        if (IsRangeValid(scan_->ranges[front_index], scan_->range_min, scan_->range_max)) {
          regions.emplace_back(front_index, i - 1);
        }
        front_index = i;
      }
    }
  }

  //拖尾滤波
  void HeavyTailedFilt(const unsigned int size = 1) {
    std::vector<Eigen::Array2i> regions;
    GetContinuousRegions(regions);
//    LOG(INFO) << "region size:" << regions.size();
    for (const auto &region: regions) {
      if (region.y() - region.x() < size) {
//        LOG(INFO) << "delta:" << region.y() - region.x();
        FiltInvalidRegion(region);
      }else{
        LOG(INFO) << "delta:" << region.y() - region.x();
      }
    }
  }

 private:
  //判断距离是否有效
  static bool IsRangeValid(const float range, const double range_min, const double range_max) {
    return (range >= range_min && range <= range_max);
  }

  //判断角度是否有效
  static bool IsAngleValid(const double angle, const double angle_min, const double angle_max) {
    return (angle >= angle_min && angle <= angle_max);
  }

  //判断序号i对应的scan点是否为断点
  bool IsBreakIndex(const unsigned int i) const {
    if (i == 0)
      return true;
    const double r1 = scan_->ranges[i - 1];
    const double r2 = scan_->ranges[i];
    const double distance = std::sqrt(r1 * r1 + r2 * r2 - 2.0 * r1 * r2 * cos_theta_);
    const double range = std::min(r1, r2);
    return distance > k_ * range + 3.0 * sigma_r_;
  }
};
}
