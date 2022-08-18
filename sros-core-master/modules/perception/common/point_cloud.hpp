/**
 * @file pointCloud.hpp
 * @brief point set class
 *
 * point set class template,
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_POINT_CLOUD_HPP
#define PERCEPTION_SOLUTION_POINT_CLOUD_HPP

// INCLUDE
#include "point3d.hpp"
#include "normal.hpp"
#include <vector>
/**
 * @brief point set class template.
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:13
 */
template<class PointT = Point3D>
class PointTCloud {
public:
    /**
     * @brief Default constructor
     */
    PointTCloud() = default;

    /**
     * @brief copy constructor
     * @param[in] rhs
     */
    PointTCloud(const PointTCloud& rhs) = default;

    /**
     * @brief Assignment constructor
     * @param[in] rhs
     * @return
     */
    PointTCloud& operator=(const PointTCloud& rhs) = default;

    /**
     * @brief add two point cloud and r
     * @param[in] input point cloud.
     */
    void add(const PointTCloud &input) {
        const size_t size = input.size();
        for (size_t i = 0; i < size; ++i) {
            push_back(input.points[i], input.image_indices[i]);
        }
    }

    /**
     * @brief Pre allocated memory space
     * @param[in] size size of memory
     */
    inline void reserve(const size_t &size) {
        points.reserve(size);
        image_indices.reserve(size);
    }

    /**
     * @brief clear point3d and point-indices.
     */
    inline void clear(){
        points.clear();
        image_indices.clear();
    }

    /**
     * @brief
     * @param[in] point point(x,y,x)
     * @param[in] index index of point cloud.
     */
    inline void push_back(const PointT &point, const unsigned int &index){
        points.emplace_back(point);
        image_indices.push_back(index);
    }

    inline void emplace_back(const PointT &point, const unsigned int &index){
        points.emplace_back(point);
        image_indices.emplace_back(index);
    }
    
    /**
     * @brief Judge whether the program is empty. If it is null, it returns true,
     *        otherwise it returns false
     * @return If points is null, it returns true, otherwise return false;
     */
    inline bool empty() const {
        return points.empty();
    }

    /**
     * @brief reset points and image_indices size.
     * @param[in] size reset size.
     */
    inline void resize(const size_t &size){ 
        points.resize(size);
        image_indices.resize(size);
    }

    /**
     * @brief get point size.
     * @return point size.
     */
    inline size_t size() const {
        return points.size();
    }

    /**
     * @brief Get the nth point.
     * @param[in] nth point.
     * @return the nth point ref.
     */
    inline const Point3D& at (size_t n) const { return (points().at (n)); }

    /**
     * @brief add a point at end of points vector, add a index of point in cloud image
     *        at image_indices vector.
     * @param idx   index of cloud image
     * @param point 3d-coordinate
     */
    inline void push(const int &idx, const Point3D &point) {
        this->indices.push_back(idx);
        this->points.push_back(point);
    }

    /** @brief time of point cloud. */
    uint64_t time;

    /** @brief the index of the point in camera image. */
    std::vector<int> image_indices;

    /** @brief Point cloud all points. */
    std::vector<PointT> points;

    typedef std::shared_ptr<PointTCloud> Ptr;
    typedef std::shared_ptr<PointTCloud const> ConstPtr;
};

struct PointIndices {
    PointIndices() : indices() {}
    PointIndices(unsigned int idx_, unsigned int cloud_point_index_)
            : idx(idx_), cloud_point_index(cloud_point_index_) {}

    unsigned int idx;
    unsigned int cloud_point_index;
    std::vector<int> indices;

    bool operator < (const PointIndices &p) const { return (idx < p.idx); }

    typedef std::shared_ptr<PointIndices> Ptr;
    typedef std::shared_ptr<PointIndices const> ConstPtr;
}; // struct PointIndices

using Indices = PointIndices;
using IndicesPtr = Indices::Ptr;
using PointCloud = PointTCloud<Point3D>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;
using NormalCloud = PointTCloud<Normal>;
using NormalCloudPtr = NormalCloud::Ptr;
using NormalCloudConstPtr = NormalCloud::ConstPtr;

#endif //PERCEPTION_SOLUTION_POINT_CLOUD_HPP
