/**
 * @file estimated_normal.h
 * @brief estimated palne parameters local
 * 
 * Compute the Least-Squares plane fit for a given set of points, and return the estimated plane
 * parameters together with the surface curvature.
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/10/28
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CLOUD_ALGORITHM_ESTIMATED_NORMAL_H
#define CLOUD_ALGORITHM_ESTIMATED_NORMAL_H

// INCLUDE
#include <vector>
#include "../../../common/point_cloud.hpp"
#include "../kd_tree/static_kd_tree_3d.hpp"

//CODE

/** \brief NormalEstimation estimates local surface properties (surface normals and curvatures)at each
 * 3D point. If PointOutT is specified as pcl::Normal, the normal is stored in the first 3 components (0-2),
 * and the curvature is stored in component 3.
 *
 * \note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
 * \ref NormalEstimationOMP for a parallel implementation.
 */
class NormalEstimation {

public:

    /** \brief Empty constructor. */
    NormalEstimation();

    /** \brief Empty destructor */
    ~NormalEstimation() = default;

    /**
     * @brief Provide a pointer to the input dataset
     * @param[in] cloud the const std shared pointer to a PointCloud message
     */
    inline void setInputCloud(const PointCloudConstPtr &cloud){
        this->input_ = cloud;
    }

    /** \brief Provide a pointer to the search object.
     * \param[in] tree a pointer to the spatial search object.
     */
    inline void setSearchMethod(const KdTree3dPtr &kd_tree){
        this->kd_tree_ = kd_tree;
    }

    /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
     * estimation.
     * \param[in] radius the sphere radius used as the maximum distance to consider a point a neighbor
     */
    inline void setRadiusSearch(const float radius){
        this->radius_ = radius;
    }

    inline void compute(NormalCloud &output){
        computeFeature(output);
    }

     /**
      * \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param[in] cloud cloud the input point cloud
      * \param[in] indices indices the point cloud indices that need to be used
      * \param[out] nx plane_normal x
      * \param[out] ny plane_normal y
      * \param[out] nz plane_normal z
      * \param[out] curvature the state of being curved
      * \return fail return false, else true
      */
    bool computePointNormal(const PointCloudConstPtr &cloud,
                            const std::vector<int> &indices,
                            float &nx, float &ny, float &nz, float &curvature);

private:

    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
     * \param[in] point a given point
     * \param[out] vp_x the X coordinate of the viewpoint
     * \param[out] vp_y the X coordinate of the viewpoint
     * \param[out] vp_z the X coordinate of the viewpoint
     * \param[out] nx the resultant X component of the plane normal
     * \param[out] ny the resultant Y component of the plane normal
     * \param[out] nz the resultant Z component of the plane normal
     * \ingroup features
     */
    inline void flipNormalTowardsViewpoint(const Point3D &point,
                                           float vp_x, float vp_y, float vp_z,
                                           float &nx, float &ny, float &nz);

    /** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
     * plane normal and surface curvature.
     * \param[in] covariance_matrix the 3x3 covariance matrix
     * \param[out] nx the resultant X component of the plane normal
     * \param[out] ny the resultant Y component of the plane normal
     * \param[out] nz the resultant Z component of the plane normal
     * \param[out] curvature the estimated surface curvature as a measure of
     * \f[
     * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
     * \f]
     * \ingroup features
     */
    void solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix,
                                     float &nx, float &ny, float &nz, float &curvature);

    /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
     * Normalized means that every entry has been divided by the number of entries in indices.
     * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
     * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
     * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
     * \param[in] cloud the input point cloud
     * \param[out] covariance_matrix the resultant 3x3 covariance matrix
     * \param[out] centroid the centroid of the set of points in the cloud
     * \return number of valid point used to determine the covariance matrix.
     * In case of dense point clouds, this is the same as the size of input cloud.
     * \ingroup common
     */
    void computeMeanAndCovarianceMatrix(const PointCloudConstPtr &cloud,
                                        const std::vector<int> &indices,
                                        Eigen::Matrix3f &covariance_matrix,
                                        Eigen::Vector4f &centroid);

    /** @brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
     * setSearchSurface () and the spatial locator in setSearchMethod ()
     * @note In situations where not enough neighbors are found, the normal and curvature values are set to NaN.
     * @param[out] output the resultant point cloud model dataset that contains surface normals and curvatures
     */
    void computeFeature(NormalCloud &output);

    /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
     * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
    float vpx_, vpy_, vpz_;

    /** \brief Placeholder for the 3x3 covariance matrix at each surface patch. */
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;

    /** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
    Eigen::Vector4f xyz_centroid_;


    /** \brief The input point cloud dataset. */
    PointCloudConstPtr input_;

    /** \brief A pointer to the spatial search object. */
    KdTree3dPtr kd_tree_;

    /** \brief number of point to fit plane. */
    int k_;

    /** \brief distance measure in the L2 Euclidean space. */
    float radius_;
};


#endif //CLOUD_ALGORITHM_ESTIMATED_NORMAL_H
