/**
 * @file euclidean_cluster_extraction.h
 * @brief euclidean cluster class
 * 
 * Decompose a region of space into clusters based on the Euclidean distance between points
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/10/28
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CLOUD_ALGORITHM_EUCLIDEAN_CLUSTER_EXTRACTION_H
#define CLOUD_ALGORITHM_EUCLIDEAN_CLUSTER_EXTRACTION_H

// INCLUDE
#include "../../../common/point_cloud.hpp"
#include "../kd_tree/static_kd_tree_3d.hpp"

//CODE
/**
 * @brief Decompose a region of space into clusters based on the Euclidean distance between points
 */
class EuclideanClusterExtraction {
public:
    /** \brief Empty constructor. */
    EuclideanClusterExtraction() :
        min_pts_per_cluster_(0),
        max_pts_per_cluster_((std::numeric_limits<int>::max)()) {}

    /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
     * \param[in] tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
     */
    inline void setClusterTolerance(const float tolerance){
        this->tolerance_ = tolerance;
    }

    /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
     * \param[in] min_cluster_size the minimum cluster size
     */
    inline void setMinClusterSize(const int min_size){
        this->min_pts_per_cluster_ = min_size;
    }

    /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
     * \param[in] max_cluster_size the maximum cluster size
     */
    inline void setMaxClusterSize(const int max_size){
        this->max_pts_per_cluster_ = max_size;
    }

    /** \brief Provide a pointer to the search object.
     * \param[in] tree a pointer to the spatial search object.
     */
    inline void setSearchMethod(const KdTree3dPtr &kdtree){
        this->kd_tree_ = kdtree;
    }

    /** \brief Provide a pointer to the input dataset
     * \param[in] cloud the const std shared pointer to a PointCloud message
     */
    inline void setInputCloud(const PointCloudConstPtr &cloud){
        this->cloud_ = cloud;
    }

    /** \brief Provide a pointer to the input dataset
     * \param[in] normals the const std shared pointer to a NormalCloud message
     */
    inline void setInputNormal(const NormalCloudPtr &normals){
        this->normals_ = normals;
    }

    /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
     * \param[out] clusters the resultant point clusters
     */
    void extract(std::vector<Indices> &clusters);

private:

    /** \brief Sort clusters method (for std::sort).
     * \ingroup segmentation
     */
    static bool comparePointClusters (const Indices &a, const Indices &b) {
        return (a.indices.size() < b.indices.size());
    }

    /** \brief The input point cloud dataset. */
    PointCloudConstPtr cloud_;

    /** \brief The input normal cloud dataset. */
    NormalCloudConstPtr normals_;

    /** \brief The maximum allowed difference between the plane normal and the given axis. */
    double eps_angle_;

    /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
    float tolerance_;

    /** \brief A pointer to the spatial search object. */
    KdTree3dPtr kd_tree_;

    /** \brief The minimum number of points that a cluster needs to contain in order to be considered valid (default = 1). */
    unsigned int min_pts_per_cluster_;

    /** \brief The maximum number of points that a cluster needs to contain in order to be considered valid (default = MAXINT). */
    unsigned int max_pts_per_cluster_;
};

#endif //CLOUD_ALGORITHM_EUCLIDEAN_CLUSTER_EXTRACTION_H
