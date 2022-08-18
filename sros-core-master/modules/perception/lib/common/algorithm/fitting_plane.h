/**
 * @file statistical_fitting_plane.h
 * @brief ransac fitting plan
 *
 * SACSegmentation represents the Nodelet segmentation class for
 * Sample Consensus methods and models, in the sense that it just creates a
 * Nodelet wrapper for generic-purpose SAC-based segmentation.
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/10/30
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CLOUD_ALGORITHM_FITTING_PALLET_PLANE_H
#define CLOUD_ALGORITHM_FITTING_PALLET_PLANE_H

// INCLUDE
#include "../../../common/point_cloud.hpp"


//CODE
/** \brief SACSegmentation represents the Nodelet segmentation class for
  * Sample Consensus methods and models, in the sense that it just creates a
  * Nodelet wrapper for generic-purpose SAC-based segmentation.
  */
class FittingPlane {
public:
    /** \brief default constructor. */
    FittingPlane() = default;

    /** \brief default deconstruction. */
    ~FittingPlane() = default;

    /** \brief Provide a pointer to the input dataset
     * \param[in] cloud the const std shared pointer to a PointCloud message
     */
    inline void setInputCloud(const PointCloudConstPtr &cloud) {
        this->cloud_ = cloud;
    }

    /** \brief Provide a pointer to the input dataset
     * \param[in] normals the const std shared pointer to a NormalCloud message
     */
    inline void setInputNormals(const NormalCloudConstPtr &normal) {
        this->normals_ = normal;
    }

    /**
     * \brief  Set the maximum allowable distance from the interior point to the model
     * \param[in] distance_threshold distance value.
     */
    inline void setDistanceThreshold(const float distance_threshold) {
        this->distance_threshold_ = distance_threshold;
    }

    /**
     * \brief Set surface normal weight coefficient
     * \param[in] normal_threshold threshold value
     */
    inline void setNormalThreshold(const float normal_threshold) {
        this->normal_threshold_ = normal_threshold;
    }

    /** \brief The registration fails if the number of iterations exceeds the maximum number of iterations.
     * \param[in] max_iterations maximum number of iterations */
    inline void setMaxIterations(const int max_iterations) {
        this->max_iterations_ = max_iterations;
    }

    /**
     * \brief Set the fit point ratio
     * \param[in] rate rate value
     */
    inline void setPointRate(const float rate){
        this->point_rate_ = rate;
    }

    /**
     * \brief Fitting plane
     * \param[out] plane_indices
     * \param[out] coefficients_plane
     */
    void fitting(
        std::vector<int> &plane_indices,
        std::vector<float> &coefficients_plane);

private:

    /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
     * Normalized means that every entry has been divided by the number of entries in indices.
     * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
     * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
     * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
     * \param[in] cloud the input point cloud
     * \param[in] indices subset of points given by their indices
     * \param[out] covariance_matrix the resultant 3x3 covariance matrix
     * \param[out] centroid the centroid of the set of points in the cloud
     * \return number of valid point used to determine the covariance matrix.
     * In case of dense point clouds, this is the same as the size of input cloud.
     * \ingroup common
     */
    void computeMeanAndCovarianceMatrix(PointCloudConstPtr &cloud,
                                        const std::vector<int> &indices,
                                        Eigen::Matrix3f &covariance_matrix,
                                        Eigen::Vector4f &centroid);

    /**
     * @brief get set of rand num.
     * @param[out] nums
     */
    inline void getRandNum(std::vector<int> &nums);

    /** \brief If the percentage of points is greater than rate, the iteration is terminated. */
    float point_rate_;

    /** \brief Maximum allowable distance of interior point. */
    float distance_threshold_;

    /** \brief The minimum and maximum normal-angel allowed opening angle of valid cone model. */
    float normal_threshold_;

    /** \brief Maximum number of iterations before giving up (user given parameter). */
    int max_iterations_;

    /** \brief The input point cloud dataset. */
    PointCloudConstPtr cloud_;

    /** \brief The input normal cloud dataset. */
    NormalCloudConstPtr normals_;
};


#endif //CLOUD_ALGORITHM_FITTING_PALLET_PLANE_H

