//
// Created by getup on 18-12-6.
//

#ifndef PROJECT_KD_TREE_2D_HPP
#define PROJECT_KD_TREE_2D_HPP

#include "nanoflann.hpp"
#include "point_cloud_2d.hpp"

namespace rack_detection {
// 2D平面kdtree
class KdTree2D {
 public:
    explicit KdTree2D(bool sorted = true) {
        params_.sorted = sorted;
        kd_tree_base_2d_.reset(new KdTreeBase2D(2, point_cloud_));
    }

 private:
    using KdTreeBase2D =
        nanoflann::KDTreeSingleIndexAdaptor<nanoflann::SO3_Adaptor<float, PointCloud2D>, PointCloud2D, 2>;
    PointCloud2D point_cloud_;
    nanoflann::SearchParams params_;
    std::unique_ptr<KdTreeBase2D> kd_tree_base_2d_;

 public:
    //获得平面点云
    const std::vector<Eigen::Vector2f> &points() const { return point_cloud_.points(); }

    //获得index对应的平面点
    const Eigen::Vector2f &operator[](const size_t index) const { return point_cloud_[index]; }

    //点的总数量
    const size_t size() const { return point_cloud_.kdtree_get_point_count(); }

    void setEpsilon(const float epsilon) { params_.eps = epsilon; }

    void setSortedResults(const bool sorted) { params_.sorted = sorted; }

    //添加新的点,配合build()函数使用
    void addNewPoint(const Eigen::Vector2f &point) { point_cloud_.addNewPoint(point); }

    //全部addNewPoint()后调用此函数建立kdtree
    void build() { kd_tree_base_2d_->buildIndex(); }

    //对点云建立kdtree
    void build(const std::vector<Eigen::Vector2f> &points) {
        point_cloud_.setPoints(points);
        kd_tree_base_2d_->buildIndex();
    }

    //清空
    void clear() {
        point_cloud_.clear();
        kd_tree_base_2d_.reset(new KdTreeBase2D(2, point_cloud_));
    }

    //k近邻搜索
    int nearestKSearch(const Eigen::Vector2f &point, int num_closest, std::vector<int> &k_indices,
                       std::vector<float> &k_sqr_distances) const {
        k_indices.resize(num_closest);
        k_sqr_distances.resize(num_closest);
        nanoflann::KNNResultSet<float, int> resultSet(num_closest);
        resultSet.init(k_indices.data(), k_sqr_distances.data());
        kd_tree_base_2d_->findNeighbors(resultSet, point.data(), params_);
        return resultSet.size();
    }

    //半径搜索
    int radiusSearch(const Eigen::Vector2f &point, float radius, std::vector<int> &k_indices,
                     std::vector<float> &k_sqr_distances) const {
        std::vector<std::pair<int, float>> indices_dist;
        indices_dist.reserve(128);
        nanoflann::RadiusResultSet<float, int> resultSet(radius * radius, indices_dist);
        kd_tree_base_2d_->findNeighbors(resultSet, point.data(), params_);
        const size_t nFound = resultSet.size();
        if (params_.sorted) {
            std::sort(indices_dist.begin(), indices_dist.end(), nanoflann::IndexDist_Sorter());
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
}  // namespace rack_detection
#endif  // PROJECT_KD_TREE_2D_HPP
