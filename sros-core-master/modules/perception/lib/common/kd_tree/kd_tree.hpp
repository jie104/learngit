#ifndef KD_TREE_HPP_
#define KD_TREE_HPP_

#include <memory>
//#include "nanoflann.hpp"
#include "modules/rack_query/rack_detection/kd_tree/nanoflann.hpp"
#include "point_cloud.hpp"

namespace data_struct {

// TODO: 对于kd树来说,使用VoxelMeanPointCloud/VoxelRecentPointCloud是否合理?
// kd树基类(包含kd树的一些基本操作)
template <typename Point3D, template <typename> class PointCloudT>
class KdTree {
 public:
  explicit KdTree() {
    params_.sorted = true;
    point_cloud_.reset(new PointCloudT<Point3D>);
  }
  virtual ~KdTree() = default;

 protected:
  std::unique_ptr<PointCloudT<Point3D>> point_cloud_;  //原始点云
  nanoflann::SearchParams params_;                    //搜索参数

 public:
  //获得原始点云
  const PclPointCloud<Point3D>& pointCloud() const {
    return point_cloud_->pointCloud();
  }

  //获得序数对应的点
  const Point3D& operator[](const size_t index) const {
    return (*point_cloud_)[index];
  }

  // TODO:如何在编译期确定是否可调用此函数
  //设置体素大小(VoxelPointCloud, VoxelMeanPointCloud,VoxelRecentPointCloud)
  void setVoxelSize(const float voxel_size) {
    point_cloud_->setVoxelSize(voxel_size);
  }

  //设置更新条件
  void setUpdateCondition(
      std::function<bool(const Point3D&, const Point3D&)> is_update) {
    point_cloud_->setUpdateCondition(is_update);
  }

  //重置整个kd树
  virtual void reset() = 0;

  //搜索最近的k个点
  virtual int nearestKSearch(const Point3D& point, int num_closest,
                             std::vector<int>& k_indices,
                             std::vector<float>& k_sqr_distances) const = 0;

  //搜索固定半径的圆内的点
  virtual int radiusSearch(const Point3D& point, float radius,
                           std::vector<int>& k_indices,
                           std::vector<float>& k_sqr_distances) const = 0;
};
}  // namespace data_struct

#endif