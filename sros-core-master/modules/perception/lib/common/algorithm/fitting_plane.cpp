/**
 * @file fitting_pallet_plane.cpp
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

// INCLUDE
#include "fitting_plane.h"
#include "estimated_normal.h"

#include <chrono>
#include <random>
#include <iostream>
#include <Eigen/Dense>

const float EPSINON = 0.00001;

// 计算两向量之间的夹角
float calculaterAngle(const Normal &u, const Normal &v){
    /** 两向量之间的夹角公式
    *                 u x v
    * cos(theta) = ———————————
    *               |U| * |v|
    */
    const float across =  u.normal_x * v.normal_x + u.normal_y * v.normal_y + u.normal_z * v.normal_z;
    const float u_len = u.normal_x * u.normal_x + u.normal_y * u.normal_y + u.normal_z * u.normal_z;
    const float v_len = v.normal_x * v.normal_x + v.normal_y * v.normal_y + v.normal_z * v.normal_z;
    return acos(across / sqrt(u_len * v_len));
}

// 计算点到平面的距离
float distanceToPlane( const Normal &normal,
                       const Point3D &center,
                       const Point3D &point){
    /// TODO: 构成平面方程
    // 平面方程：一般式 Ax + By + Cz + D = 0; 点法式： A(x - x0) + B(y - y0) + C(z - z0) = 0;
    const float A = normal.normal_x;
    const float B = normal.normal_y;
    const float C = normal.normal_z;
    const float D = -(A * center.x + B * center.y + C * center.z);

    /** 点到平面的距离
     *     | A*x + B*y + C*z + D |
     * d = ————————————————————————
     *      sqrt(A*A + B*B + C*C)
     */
    float molecule = fabs(A * point.x + B * point.y + C * point.z + D);
    float denominator = sqrt(A*A + B*B + C*C);

    return molecule / denominator;
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

// CODE
void FittingPlane::fitting(
    std::vector<int> &plane_indices,
    std::vector<float> &coefficients_plane) {

    if (normals_->points.empty() || cloud_->points.empty()){
        coefficients_plane.clear();
        return;
    }

    assert(normals_->size() == cloud_->size());

    const int size = cloud_->size();
    int N = size;

    N = N*0.1 < 5 ? 5 : N*0.1;
    N = N > 50 ? 50 : N;

    plane_indices.resize(0);

    // 随机数种子生成器
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed);  // 大随机数
    std::uniform_int_distribution<int> dis(0, size-1);  // 给定范围[0, size-1]
    int *flag = new int[size];

    auto getRandInner = [&](const int N, std::vector<int> &inner) {
        int number;
        memset(flag, 0 , size*sizeof(int));
        inner.resize(N);
        size_t i = 0, j= 0;
        while(i < N){
            number = dis(gen); // 随机生成一个点
            if (!flag[number]) {
                flag[number] = 1;
                inner[j] = number;
                j++;
                i++;
            }
        }
    };

    float nx, ny, nz, curvature;
    Normal plane_normal;
    Point3D plane_center;
    Eigen::Vector4f xyz_centroid;
    std::vector<int> inner_indices;
    NormalEstimation ne;
    float A,B,C,D;
    const Point3D *a, *b, *c;
    for (size_t i = 0; i < max_iterations_; ++i) {

        /// TODO: 产生一组[0～size-1]的随机数当成局内点
        getRandInner(N, inner_indices);

        /// TODO: 计算这组随机点所组成的平面
        ne.computePointNormal(cloud_, inner_indices, nx, ny, nz, curvature);
        plane_center.x = plane_center.y = plane_center.z = 0;
        for (auto idx : inner_indices){
            plane_center.x += cloud_->points.at(idx).x;
            plane_center.y += cloud_->points.at(idx).y;
            plane_center.z += cloud_->points.at(idx).z;
        }

        // 平面重心
        plane_center.x /= N;
        plane_center.y /= N;
        plane_center.z /= N;

        // 平面法向量
        plane_normal.normal_x = nx;
        plane_normal.normal_y = ny;
        plane_normal.normal_z = nz;

        /// TODO: 把符合平面方程的局外点加入到局内点
        for (size_t j = 0; j < size; ++j){

            // 如果当前点已经是局内点则跳过,否则判断该点是否符合平面模型。
            if (flag[j]) continue;

            // 如果两法向量之间的夹角大于阀值则放弃
            if ( M_PI-fabs(calculaterAngle(normals_->points.at(j),  plane_normal)) > normal_threshold_){
                continue;
            }

            // 如果点到平面的距离大于阀值则放弃
            if (distanceToPlane(plane_normal, plane_center, cloud_->points.at(j)) > distance_threshold_){
                continue;
            }

            inner_indices.push_back(j);
            flag[j] = 1;
        }

        /// TODO: 选择数量最多的一组局内点
        // 可以自定义拟合质量公式,用夹角平均值和点到平面的距离距离和内点数量三个参数和权重系数（系统和为1）评估拟合质量,
        //  Q = (avg_angle, avg_dist, size) x (weight_angle, weight_dist, weight_size)
        if (inner_indices.size() > plane_indices.size()){
            plane_indices.resize(inner_indices.size());
            copy(inner_indices.begin(), inner_indices.end(), plane_indices.begin());
        }

        if ((inner_indices.size() / size) > this->point_rate_)
            break;
    }

    delete []flag;

    /// TODO: 所有局内点重新拟合平面
    ne.computePointNormal(cloud_, plane_indices, nx, ny, nz, curvature);

    plane_center.x = plane_center.y = plane_center.z = 0;
    for (auto idx : plane_indices){
        plane_center.x += cloud_->points.at(idx).x;
        plane_center.y += cloud_->points.at(idx).y;
        plane_center.z += cloud_->points.at(idx).z;
    }

    // 平面重心
    plane_center.x /= plane_indices.size();
    plane_center.y /= plane_indices.size();
    plane_center.z /= plane_indices.size();

    coefficients_plane.resize(4);
    coefficients_plane[0] = nx;
    coefficients_plane[1] = ny;
    coefficients_plane[2] = nz;
    coefficients_plane[3] = -(nx * plane_center.x + ny * plane_center.y + nz * plane_center.z);
}