#ifndef IMAGE_RASTERIZER_HPP_
#define IMAGE_RASTERIZER_HPP_

#include "math.hpp"
#include "point_type.hpp"

namespace obstacle_detection {
class ImageRasterizer {
 public:
  ImageRasterizer() {
    point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }

 private:
  static constexpr unsigned int height_ = 172;
  static constexpr unsigned int width_ = 224;
  static constexpr unsigned int bound_ = 8;
  static constexpr float max_intensity_ = 768.f;
  static constexpr float max_curvature_ = 1.f;
  static constexpr float max_delta_x_ = 0.015f;

  std::array<std::array<RasterizedPoint, width_>, height_> image_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_point_cloud_;
  std::vector<Eigen::Vector2f> obstacle_points_;

 public:
  void process(const cv::Mat& confidence_img, const cv::Mat& xyz_img,
               const cv::Mat& amplitude_img) {
    getImage(confidence_img, xyz_img, amplitude_img);
    getFrontAndBackIndex();
    getScore();
    getObstacle();
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud() const {
    return point_cloud_;
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredPointCloud() const {
    return filtered_point_cloud_;
  }

  const std::vector<Eigen::Vector2f>& ObstaclePoints() const {
    return obstacle_points_;
  }

 private:
  void getImage(const cv::Mat& confidence_img, const cv::Mat& xyz_img,
                const cv::Mat& amplitude_img) {
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        auto& point = image_[row][col];
        const unsigned char confidence =
            confidence_img.at<unsigned char>(row, col);
        const std::bitset<8> bits(confidence);
        if (bits[0] != 1 && bits[1] != 1 && bits[2] != 1 && bits[3] != 1) {
          point.validity = true;
          point.x = xyz_img.at<cv::Vec3f>(row, col)[0];
          point.y = xyz_img.at<cv::Vec3f>(row, col)[1];
          point.z = xyz_img.at<cv::Vec3f>(row, col)[2];
          point.intensity = amplitude_img.at<float>(row, col);
        } else {
          point.validity = false;
        }
      }
    }
  }

  void getFrontAndBackIndex() {
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        auto& point = image_[row][col];
        if (point.validity) {
          int valid_pos = 0;
          for (int pos = 1; pos < bound_ && row - pos >= 0; pos++) {
            if (image_[row - pos][col].validity) {
              valid_pos = pos;
              if (std::abs(image_[row - pos][col].x - point.x) >=
                  max_delta_x_) {
                break;
              }
            }
          }
          point.front_index = row - valid_pos;
          valid_pos = 0;
          for (int pos = 1; pos < bound_ && row + pos < height_; pos++) {
            if (image_[row + pos][col].validity) {
              valid_pos = pos;
              if (std::abs(image_[row + pos][col].x - point.x) >=
                  max_delta_x_) {
                break;
              }
            }
          }
          point.back_index = row + valid_pos;
        }
      }
    }
  }

  void getScore() {
    point_cloud_->clear();
    pcl::PointXYZI point_xyzi;
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        auto& p = image_[row][col];
        if (p.validity && p.front_index != row && p.back_index != row) {
          const auto& pf = image_[p.front_index][col];
          const auto& pb = image_[p.back_index][col];
          float curvature = 0.f;
          const float alpha_f =
              std::atan(std::abs(pf.z - p.z) / std::abs(pf.x - p.x));
          const float alpha_b =
              std::atan(std::abs(pb.z - p.z) / std::abs(pb.x - p.x));
          const float z_mean = (pf.z + pb.z) * 0.5f;
          if (p.z < z_mean) {
            curvature = std::min(std::abs(alpha_f), std::abs(alpha_b));
          } else {
            curvature = std::abs(alpha_f - alpha_b);
          }

          float intensity_delta_rate = 1.f;
          if (p.intensity < max_intensity_) {
            intensity_delta_rate =
                std::max(std::abs(pf.intensity - p.intensity) /
                             std::max(pf.intensity, p.intensity),
                         std::abs(pb.intensity - p.intensity) /
                             std::max(pb.intensity, p.intensity));
          }
          const float score_d = std::min(1.f, curvature / max_curvature_);
          const float score_i = intensity_delta_rate;
          float score = 0.f;
          if (score_d != 0.f || score_i != 0.f) {
            score = score_d * score_i / (score_d * 0.7f + score_i * 0.3f);
          }
          point_xyzi.x = p.x;
          point_xyzi.y = p.y;
          point_xyzi.z = p.z;
          point_xyzi.intensity = score;
          point_cloud_->push_back(point_xyzi);
        }
      }
    }
  }

  void getObstacle() {
    filtered_point_cloud_->clear();
    obstacle_points_.clear();
    for (const auto& point : *point_cloud_) {
      if (point.intensity < 0.6f) continue;
      if (point.x < 0.3f) continue;
      if (point.x > 1.2f) continue;
      if (std::abs(point.y) > point.x * 0.5f) continue;
      filtered_point_cloud_->push_back(point);
      obstacle_points_.emplace_back(point.x, point.y);
    }
  }
};
}  // namespace obstacle_detection
#endif