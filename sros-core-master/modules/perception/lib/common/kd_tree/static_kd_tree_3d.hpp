#ifndef STATIC_KD_TREE_3D_HPP_
#define STATIC_KD_TREE_3D_HPP_

#include "kd_tree.hpp"

namespace data_struct {
//静态三维搜索kd树
template <typename Point3D, template <typename> class PointCloudT>
class StaticKdTree3d : public KdTree<Point3D, PointCloudT> {
 public:
  explicit StaticKdTree3d() {
    static_kd_tree_base_.reset(new KdTreeBaseT(3, *point_cloud_));
  }

  typedef std::shared_ptr<StaticKdTree3d> Ptr;
  typedef std::shared_ptr<StaticKdTree3d const> ConstPtr;

 private:
  using KdTree<Point3D, PointCloudT>::point_cloud_;
  using KdTree<Point3D, PointCloudT>::params_;
  using KdTreeBaseT = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::SO3_Adaptor<float, PointCloudT<Point3D>>, PointCloudT<Point3D>,
      3>;
  std::unique_ptr<KdTreeBaseT> static_kd_tree_base_;

 public:
  //添加新的数据
  void add(const Point3D& point) { point_cloud_->add(point); }

  //建立kd树(根据之前添加的数据)
  void build() { static_kd_tree_base_->buildIndex(); }

  //对点云数据建立kd树
  void build(const PclPointCloud<Point3D>& point_cloud) {
    if(point_cloud.size() > 0) {
        point_cloud_->set(point_cloud);
        static_kd_tree_base_->buildIndex();
    }
  }

  void reset() override {
    point_cloud_->clear();
    static_kd_tree_base_.reset(new KdTreeBaseT(3, *point_cloud_));
  }

  int nearestKSearch(const Point3D& point, int num_closest,
                     std::vector<int>& k_indices,
                     std::vector<float>& k_sqr_distances) const override {
    k_indices.resize(num_closest);
    k_sqr_distances.resize(num_closest);
    nanoflann::KNNResultSet<float, int> resultSet(num_closest);
    resultSet.init(k_indices.data(), k_sqr_distances.data());
    static_kd_tree_base_->findNeighbors(resultSet, point.data, params_);
    return resultSet.size();
  }

  int radiusSearch(const Point3D& point, float radius,
                   std::vector<int>& k_indices,
                   std::vector<float>& k_sqr_distances) const override {
    std::vector<std::pair<int, float>> indices_dist;
    indices_dist.reserve(128);
    nanoflann::RadiusResultSet<float, int> resultSet(radius * radius,
                                                     indices_dist);
    static_kd_tree_base_->findNeighbors(resultSet, point.data, params_);
    const size_t nFound = resultSet.size();
    if (params_.sorted) {
      std::sort(indices_dist.begin(), indices_dist.end(),
                nanoflann::IndexDist_Sorter());
    }
    k_indices.resize(nFound);
    k_sqr_distances.resize(nFound);
    for (int i = 0; i < nFound; i++) {
      k_indices[i] = indices_dist[i].first;
      k_sqr_distances[i] = indices_dist[i].second;
    }
    return nFound;
  }
};
}  // namespace data_struct

using KdTree3d = data_struct::StaticKdTree3d<Point3D, point_cloud::PointCloud>;
using KdTree3dPtr = KdTree3d::Ptr;

#endif