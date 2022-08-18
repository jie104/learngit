/**
 * @file radius_filter.h
 * @brief outlier removal
 * @author wuchaohuo@standard-robots.com
 * @date create date：2022/04/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CLOUD_ALGORITHM_RADIUS_FILTER_H
#define CLOUD_ALGORITHM_RADIUS_FILTER_H

// INCLUDE
#include "../../../common/point_cloud.hpp"
#include "../kd_tree/static_kd_tree_3d.hpp"

//CODE
/** \brief RadiusFilter uses point neighborhood statistics to filter outlier data. */
class RadiusFilter {

public:
    
    RadiusFilter(bool extract_removed_indices = false) :
        search_radius_(0.0),
        min_pts_radius_(1){
    }

  
    ~RadiusFilter() = default;

    /** \brief Provide a pointer to the search object.
     * \param[in] tree a pointer to the spatial search object.
     */
    inline void setKdTree(const KdTree3dPtr &kdtree){
        this->kd_tree_ = kdtree;
    }

    /**
     * @brief Provide a pointer to the input dataset
     * @param[in] cloud the const std shared pointer to a PointCloud message
     */
    inline void setInputCloud(const PointCloudConstPtr &input){
        this->input_ = input;
    }

    /** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors for filtering.
    * \param radius the sphere radius that is to contain all k-nearest neighbors
    */
    inline void
    setRadiusSearch (double radius)
    {
        search_radius_ = radius;
    }

    /** \brief Set the minimum number of neighbors that a point needs to have in the given search radius in order to
    * be considered an inlier (i.e., valid).
    * \param min_pts the minimum number of neighbors
    */
    inline void
    setMinNeighborsInRadius (int min_pts)
    {
        min_pts_radius_ = min_pts;
    }

    /** \brief Get the minimum number of neighbors that a point needs to have in the given search radius to be
    * considered an inlier and avoid being filtered.
    */
    inline double
    getMinNeighborsInRadius ()
    {
        return (min_pts_radius_);
    }

    /** \brief Filtered results are indexed by an indices array.
      * \param[out] indices The resultant indices.
      */
    void filter(std::vector<int>& indices){
        applyFilterIndices(indices);
    }

private:
    /** \brief Filtered results are indexed by an indices array.
     * \param[out] indices The resultant indices.
     */
    void applyFilterIndices(std::vector<int>& indices);

    /** \brief The number of points to use for mean distance estimation. */
    double search_radius_;

    /** \brief Standard deviations threshold (i.e., points outside of*/
    int min_pts_radius_;

    /** \brief A pointer to the spatial search object. */
    KdTree3dPtr kd_tree_;

    /** \brief The input point cloud dataset. */
    PointCloudConstPtr input_;
};


#endif //CLOUD_ALGORITHM_RADIUS_FILTER_H
