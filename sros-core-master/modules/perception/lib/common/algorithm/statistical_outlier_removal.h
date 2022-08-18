/**
 * @file statiscal_outlier_removal.h
 * @brief outlier removal
 * 
 * The algorithm iterates through the entire input twice:
 * During the first iteration it will compute the average distance that each point
 * has to its nearest k neighbors.
 * The value of k can be set using setMeanK().
 * Next, the mean and standard deviation of all these distances are computed in order
 * to determine a distance threshold.
 * The distance threshold will be equal to: mean + stddev_mult * stddev.
 * The multiplier for the standard deviation can be set using setStddevMulThresh().
 * During the next iteration the points will be classified as inlier or outlier if their
 * average neighbor distance is below or above this threshold respectively.
 * <br>
 * The neighbors found for each query point will be found amongst ALL points of setInputCloud(),
 * not just those indexed by setIndices().
 * The setIndices() method only indexes the points that will be iterated through as search query points.
 * <br><br>
 * For more information:
 *   - R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
 *     Towards 3D Point Cloud Based Object Maps for Household Environments
 *     Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
 * <br><br>
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/10/28
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CLOUD_ALGORITHM_STATISTICAL_OUTLIER_REMOVAL_H
#define CLOUD_ALGORITHM_STATISTICAL_OUTLIER_REMOVAL_H

// INCLUDE
#include "../../../common/point_cloud.hpp"
#include "../kd_tree/static_kd_tree_3d.hpp"

//CODE
/** \brief StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data. */
class StatisticalOutlierRemoval {

public:
    /** \brief Constructor.
      * \param[in] extract_removed_indices Set to true if you want to be able to extract
      *            the indices of points being removed (default = false).
      */
    StatisticalOutlierRemoval(bool extract_removed_indices = false) :
        mean_k_(1),
        std_mul_(0.0) {
    }

    /** \brief Empty destructor */
    ~StatisticalOutlierRemoval() = default;

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

    /** \brief Set the number of nearest neighbors to use for mean distance estimation.
      * \param[in] nr_k The number of points to use for mean distance estimation.
      */
    inline void setMeanK(int nr_k) {
        mean_k_ = nr_k;
    }

    /** \brief Get the number of nearest neighbors to use for mean distance estimation.
      * \return The number of points to use for mean distance estimation.
      */
    inline int getMeanK() {
        return (mean_k_);
    }

    /** \brief Set the standard deviation multiplier for the distance threshold calculation.
      * \details The distance threshold will be equal to: mean + stddev_mult * stddev.
      * Points will be classified as inlier or outlier if their average neighbor distance
      * is below or above this threshold respectively.
      * \param[in] stddev_mult The standard deviation multiplier.
      */
    inline void setStddevMulThresh(double stddev_mult) {
        std_mul_ = stddev_mult;
    }

    /** \brief Get the standard deviation multiplier for the distance threshold calculation.
      * \details The distance threshold will be equal to: mean + stddev_mult * stddev.
      * Points will be classified as inlier or outlier if their average neighbor distance
      * is below or above this threshold respectively.
      */
    inline double getStddevMulThresh() {
        return (std_mul_);
    }

    /** \brief Filtered results are indexed by an indices array.
      * \param[out] indices The resultant indices.
      */
    void filter(std::vector<int> &indices){
        applyFilterIndices(indices);
    }

private:
    /** \brief Filtered results are indexed by an indices array.
     * \param[out] indices The resultant indices.
     */
    void applyFilterIndices(std::vector<int> &indices);

    /** \brief The number of points to use for mean distance estimation. */
    int mean_k_;

    /** \brief Standard deviations threshold (i.e., points outside of
      * \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). */
    double std_mul_;

    /** \brief A pointer to the spatial search object. */
    KdTree3dPtr kd_tree_;

    /** \brief The input point cloud dataset. */
    PointCloudConstPtr input_;
};


#endif //CLOUD_ALGORITHM_STATISTICAL_OUTLIER_REMOVAL_H
