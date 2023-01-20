//
// Created by getup on 18-12-6.
//

#ifndef PROJECT_POINT_CLOUD_2D_HPP
#define PROJECT_POINT_CLOUD_2D_HPP

#include <Eigen/Dense>

namespace rack_detection {
//平面点云类
class PointCloud2D {
 public:
  PointCloud2D() = default;
  PointCloud2D(const PointCloud2D &rhs) = default;
  PointCloud2D &operator=(const PointCloud2D &rhs) = default;
 private:
  std::vector<Eigen::Vector2f> points_;

 public:
  const std::vector<Eigen::Vector2f> &points() const { return points_; }

  const Eigen::Vector2f &operator[](const size_t index) const {
    return points_[index];
  }

  const size_t kdtree_get_point_count() const { return points_.size(); }

  float kdtree_get_pt(const size_t index, int dim) const {
    const Eigen::Vector2f &p = points_[index];
    if (dim == 0) {
      return p.x();
    } else if (dim == 1) {
      return p.y();
    }
    return 0.0;
  }

  template<class BBOX>
  bool kdtree_get_bbox(BBOX &) const {
    return false;
  }

  void setPoints(const std::vector<Eigen::Vector2f> &points) {
    points_ = points;
  }

  void addPoints(const std::vector<Eigen::Vector2f> &points) {
    points_.insert(points_.end(), points.begin(), points.end());
  }

  void addNewPoint(const Eigen::Vector2f &point) {
    points_.push_back(point);
  }

  void clear() { points_.clear(); }
};
}

#endif //PROJECT_POINT_CLOUD_2D_HPP
