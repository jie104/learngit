/**
 * @file estimated_normal.cpp
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
// INCLUDE
#include "estimated_normal.h"
#include "Eigen/Dense"
#include "Eigen/Core"

// CODE

NormalEstimation::NormalEstimation()
    : vpx_ (0)
    , vpy_ (0)
    , vpz_ (0)
    , k_(0)
    , covariance_matrix_ (){

}

void NormalEstimation::computeFeature (NormalCloud &output) {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices(k_);
    std::vector<float> nn_dists(k_);
    const int size = input_->size();
    output.resize(size);
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < size; ++idx) {
        if (this->kd_tree_->radiusSearch(input_->points[idx], radius_, nn_indices, nn_dists) == 0 ||
            !computePointNormal(input_, nn_indices, output.points[idx].normal[0], output.points[idx].normal[1],
                                output.points[idx].normal[2], output.points[idx].curvature)) {
            output.points[idx].normal[0] = std::numeric_limits<float>::quiet_NaN();
            output.points[idx].normal[1] = std::numeric_limits<float>::quiet_NaN();
            output.points[idx].normal[2] = std::numeric_limits<float>::quiet_NaN();
            output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();

            continue;
        }

        flipNormalTowardsViewpoint(input_->points[idx], vpx_, vpy_, vpz_, output.points[idx].normal[0],
                                   output.points[idx].normal[1], output.points[idx].normal[2]);
    }

    output.image_indices.resize(size);
    copy(this->input_->image_indices.begin(), this->input_->image_indices.end(), output.image_indices.begin());
}

bool
NormalEstimation::computePointNormal (const PointCloudConstPtr &cloud,
                                      const std::vector<int> &indices,
                                      float &nx, float &ny, float &nz, float &curvature) {

    // Three points that are not collinear determine a plane.
    if (indices.size() < 3 ) {
        nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN();
        return false;
    }

    // Calculate the covariance matrix and mean value of the given 3D point cloud
    computeMeanAndCovarianceMatrix(cloud, indices, covariance_matrix_, xyz_centroid_);

//    std::cout << xyz_centroid_ << std::endl;
//    std::cout << covariance_matrix_ << std::endl;

    // Get the plane normal and surface curvature
    solvePlaneParameters(covariance_matrix_, nx, ny, nz, curvature);
    return true;
}


// pcl实现
//////////////////////////////////////////////////////////////////////////////////////
//void
//NormalEstimation::computeMeanAndCovarianceMatrix(const PointCloudPtr &cloud,
//                                                 const std::vector<int> &indices,
//                                                 Eigen::Matrix3f &covariance_matrix,
//                                                 Eigen::Vector4f &centroid) {
//    // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
//    Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
//    size_t point_count = indices.size();
//    for (std::vector<int>::const_iterator iIt = indices.begin(); iIt != indices.end(); ++iIt) {
//        //const Point3D& point = cloud[*iIt];
//        accu[0] += cloud->points[*iIt].x * cloud->points[*iIt].x;
//        accu[1] += cloud->points[*iIt].x * cloud->points[*iIt].y;
//        accu[2] += cloud->points[*iIt].x * cloud->points[*iIt].z;
//        accu[3] += cloud->points[*iIt].y * cloud->points[*iIt].y;
//        accu[4] += cloud->points[*iIt].y * cloud->points[*iIt].z;
//        accu[5] += cloud->points[*iIt].z * cloud->points[*iIt].z;
//        accu[6] += cloud->points[*iIt].x;
//        accu[7] += cloud->points[*iIt].y;
//        accu[8] += cloud->points[*iIt].z;
//    }
//
//    accu /= static_cast<float> (point_count);
//    //Eigen::Vector3f vec = accu.tail<3> ();
//    //centroid.head<3> () = vec;//= accu.tail<3> ();
//    //centroid.head<3> () = accu.tail<3> ();    -- does not compile with Clang 3.0
//    centroid[0] = accu[6];
//    centroid[1] = accu[7];
//    centroid[2] = accu[8];
//    centroid[3] = 1;
//    covariance_matrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
//    covariance_matrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
//    covariance_matrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
//    covariance_matrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
//    covariance_matrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
//    covariance_matrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
//    covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
//    covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
//    covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
//}

// zx实现
//////////////////////////////////////////////////////////////////////////////////////
void
NormalEstimation::computeMeanAndCovarianceMatrix( const PointCloudConstPtr &cloud,
                                                  const std::vector<int> &indices,
                                                  Eigen::Matrix3f &covariance_matrix,
                                                  Eigen::Vector4f &centroid) {
    // Calculate the coordinates of the center point
    /* The mean value of aVu x g
     * avg_x = sum(xi) / n
    * */
    const int size = indices.size();
    centroid[0] = centroid[1] = centroid[2] = 0;
    for (auto idx : indices)
    {
        centroid[0] += cloud->points[idx].x;
        centroid[1] += cloud->points[idx].y;
        centroid[2] += cloud->points[idx].z;
    }

    centroid[0] /= size;
    centroid[1] /= size;
    centroid[2] /= size;

    // Calculate the standard deviation
    /* standard deviation cov(x,y)
     *              sum( (Xi-avg_X) * (Yi-avg_y) )
     * cov(x,y) = ——————————————————————————————————
     *                          n - 1
     * */
    double x, y, z, xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (auto idx : indices){
        x = cloud->points[idx].x - centroid[0];
        y = cloud->points[idx].y - centroid[1];
        z = cloud->points[idx].z - centroid[2];
        xx += x * x;
        yy += y * y;
        zz += z * z;
        xy += x * y;
        xz += x * z;
        yz += y * z;
    }

    // Calculate the covariance matrix
    /* covariance matrix C
     *     | cov(x,x) cov(x,y) cov(x,z) |
     * C = | cov(y,x) cov(y,y) cov(y,z) |
     *     | cov(z,x) cov(z,y) cov(z,z) |
     * */
//    covariance_matrix<<1,2,3,3,3,3,3,3;
    covariance_matrix(0, 0) = xx / (size-1);
    covariance_matrix(0, 1) = covariance_matrix(1, 0) = xy / (size-1);
    covariance_matrix(0, 2) = covariance_matrix(2, 0) = xz / (size-1);
    covariance_matrix(1, 1) = yy / (size-1);
    covariance_matrix(1, 2) = covariance_matrix(2, 1) = yz / (size-1);
    covariance_matrix(2, 2) = zz / (size-1);

}

// lfc实现
//////////////////////////////////////////////////////////////////////////////////////
//void
//NormalEstimation::computeMeanAndCovarianceMatrix(const PointCloudPtr &cloud,
//                                                 const std::vector<int> &indices,
//                                                 Eigen::Matrix3f &covariance_matrix,
//                                                 Eigen::Vector4f &centroid) {
//    // 计算中心点坐标
//    /* x的均值avg_x
//     * avg_x = sum(xi) / n
//    * */
//    const int size = indices.size();
//    centroid[0] = centroid[1] = centroid[2] = 0;
//    for(const auto& indice: indices){
//        centroid[0] += cloud->points[indice].x;
//        centroid[1] += cloud->points[indice].y;
//        centroid[2] += cloud->points[indice].z;
//    }
//
//    centroid[0] /= size;
//    centroid[1] /= size;
//    centroid[2] /= size;
//
//    // 计算标准差
//    /* 标准差cov(x,y)
//     *              sum( (Xi-avg_X) * (Yi-avg_y) )
//     * cov(x,y) = ——————————————————————————————————
//     *                          n - 1
//     * */
//    double x, y, z, xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
//    for(const auto& indice: indices) {
//        x = cloud->points[indice].x - centroid[0];
//        y = cloud->points[indice].y - centroid[1];
//        z = cloud->points[indice].z - centroid[2];
//        xx += x * x;
//        yy += y * y;
//        zz += z * z;
//        xy += x * y;
//        xz += x * z;
//        yz += y * z;
//    }
//
//    // 计算协方差矩阵
//    /* 协方差矩阵C
//     *     | cov(x,x) cov(x,y) cov(x,z) |
//     * C = | cov(y,x) cov(y,y) cov(y,z) |
//     *     | cov(z,x) cov(z,y) cov(z,z) |
//     * */
////    covariance_matrix<<1,2,3,3,3,3,3,3;
//    covariance_matrix(0, 0) = xx / (size-1);
//    covariance_matrix(0, 1) = covariance_matrix(1, 0) = xy / (size-1);
//    covariance_matrix(0, 2) = covariance_matrix(2, 0) = xz / (size-1);
//    covariance_matrix(1, 1) = yy / (size-1);
//    covariance_matrix(1, 2) = covariance_matrix(2, 1) = yz / (size-1);
//    covariance_matrix(2, 2) = zz / (size-1);
//
//}


// 启杨eigen实现
//void
//NormalEstimation::computeMeanAndCovarianceMatrix(const PointCloudPtr &cloud,
//                                                 const std::vector<int> &indices,
//                                                 Eigen::Matrix3f &covariance_matrix,
//                                                 Eigen::Vector4f &centroid) {
//    const int size = indices.size();
//    const float inv_size = 1.f / static_cast<float>(size);
//    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
//    Eigen::Matrix3f sq_sum = Eigen::Matrix3f::Zero();
//    for (int i = 0; i < indices.size(); i++) {
//       const auto& point = cloud->points[indices[i]].point();
//        sum += point;
//       sq_sum += point * point.transpose();
//    }
//    centroid.head<3>() = sum * inv_size;
//    covariance_matrix = (sq_sum - sum * sum.transpose() * inv_size) / static_cast<float>(size - 1);
//}

template <typename Scalar, typename Roots> inline void
computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots)
{
    roots (0) = Scalar (0);
    Scalar d = Scalar (b * b - 4.0 * c);
    if (d < 0.0)  // no real roots ! THIS SHOULD NOT HAPPEN!
        d = 0.0;

    Scalar sd = ::std::sqrt (d);

    roots (2) = 0.5f * (b + sd);
    roots (1) = 0.5f * (b - sd);
}

template <typename Matrix, typename Roots> inline void
computeRoots (const Matrix& m, Roots& roots) {
    typedef typename Matrix::Scalar Scalar;

    // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
    // eigenvalues are the roots to this equation, all guaranteed to be
    // real-valued, because the matrix is symmetric.
    Scalar c0 = m(0, 0) * m(1, 1) * m(2, 2) +
                Scalar(2) * m(0, 1) * m(0, 2) * m(1, 2) -
                m(0, 0) * m(1, 2) * m(1, 2) -
                m(1, 1) * m(0, 2) * m(0, 2) -
                m(2, 2) * m(0, 1) * m(0, 1);
    Scalar c1 = m(0, 0) * m(1, 1) -
                m(0, 1) * m(0, 1) +
                m(0, 0) * m(2, 2) -
                m(0, 2) * m(0, 2) +
                m(1, 1) * m(2, 2) -
                m(1, 2) * m(1, 2);
    Scalar c2 = m(0, 0) + m(1, 1) + m(2, 2);

    if (fabs(c0) < Eigen::NumTraits<Scalar>::epsilon())  // one root is 0 -> quadratic equation
        computeRoots2(c2, c1, roots);
    else {
        const Scalar s_inv3 = Scalar(1.0 / 3.0);
        const Scalar s_sqrt3 = std::sqrt(Scalar(3.0));
        // Construct the parameters used in classifying the roots of the equation
        // and in solving the equation for the roots in closed form.
        Scalar c2_over_3 = c2 * s_inv3;
        Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
        if (a_over_3 > Scalar(0))
            a_over_3 = Scalar(0);

        Scalar half_b = Scalar(0.5) * (c0 + c2_over_3 * (Scalar(2) * c2_over_3 * c2_over_3 - c1));

        Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
        if (q > Scalar(0))
            q = Scalar(0);

        // Compute the eigenvalues by solving for the roots of the polynomial.
        Scalar rho = std::sqrt(-a_over_3);
        Scalar theta = std::atan2(std::sqrt(-q), half_b) * s_inv3;
        Scalar cos_theta = std::cos(theta);
        Scalar sin_theta = std::sin(theta);
        roots(0) = c2_over_3 + Scalar(2) * rho * cos_theta;
        roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
        roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

        // Sort in increasing order.
        if (roots(0) >= roots(1))
            std::swap(roots(0), roots(1));
        if (roots(1) >= roots(2)) {
            std::swap(roots(1), roots(2));
            if (roots(0) >= roots(1))
                std::swap(roots(0), roots(1));
        }

        if (roots(0) <= 0)  // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
            computeRoots2(c2, c1, roots);
    }
}

template <typename Matrix, typename Vector> inline void
eigen33 (const Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector)
{
    typedef typename Matrix::Scalar Scalar;
    // Scale the matrix so its entries are in [-1,1].  The scaling is applied
    // only when at least one matrix entry has magnitude larger than 1.

    Scalar scale = mat.cwiseAbs ().maxCoeff ();
    if (scale <= std::numeric_limits < Scalar > ::min ())
        scale = Scalar (1.0);

    Matrix scaledMat = mat / scale;

    Vector eigenvalues;
    computeRoots (scaledMat, eigenvalues);

    eigenvalue = eigenvalues (0) * scale;

    scaledMat.diagonal ().array () -= eigenvalues (0);

    Vector vec1 = scaledMat.row (0).cross (scaledMat.row (1));
    Vector vec2 = scaledMat.row (0).cross (scaledMat.row (2));
    Vector vec3 = scaledMat.row (1).cross (scaledMat.row (2));

    Scalar len1 = vec1.squaredNorm ();
    Scalar len2 = vec2.squaredNorm ();
    Scalar len3 = vec3.squaredNorm ();

    if (len1 >= len2 && len1 >= len3)
        eigenvector = vec1 / std::sqrt (len1);
    else if (len2 >= len1 && len2 >= len3)
        eigenvector = vec2 / std::sqrt (len2);
    else
        eigenvector = vec3 / std::sqrt (len3);
}

void
NormalEstimation::solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                                  float &nx, float &ny, float &nz, float &curvature) {
    // Extract the smallest eigenvalue and its eigenvector
    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;

    //  确定对称半正定输入矩阵给定特征值对应的特征向量
    eigen33(covariance_matrix, eigen_value, eigen_vector);

    nx = eigen_vector[0];
    ny = eigen_vector[1];
    nz = eigen_vector[2];

    // Compute the curvature surface change
    float eig_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);
    if (eig_sum != 0)
        curvature = fabsf(eigen_value / eig_sum);
    else
        curvature = 0;
}

/** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
* \param point a given point
* \param vp_x the X coordinate of the viewpoint
* \param vp_y the X coordinate of the viewpoint
* \param vp_z the X coordinate of the viewpoint
* \param nx the resultant X component of the plane normal
* \param ny the resultant Y component of the plane normal
* \param nz the resultant Z component of the plane normal
* \ingroup features
*/
inline void
NormalEstimation::flipNormalTowardsViewpoint (
    const Point3D &point,
    float vp_x, float vp_y, float vp_z,
    float &nx, float &ny, float &nz){
    // See if we need to flip any plane normals
    vp_x -= point.x;
    vp_y -= point.y;
    vp_z -= point.z;

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

    // Flip the plane normal
    if (cos_theta < 0)
    {
        nx *= -1;
        ny *= -1;
        nz *= -1;
    }
}