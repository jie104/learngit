/**
 * @file normal.hpp
 * @brief point normal
 *
 * The direction perpendicular to the surface at a point on the surface
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_NORMAL_HPP
#define PERCEPTION_SOLUTION_NORMAL_HPP

// INCLUDE
#include <Eigen/Dense>
#include <memory>

#define ADD_UNION_NORMAL4D  \
  struct EIGEN_ALIGN16 {     \
    float data_n[4];        \
    float normal[3];        \
    struct {                \
      float normal_x;       \
      float normal_y;       \
      float normal_z;       \
    };                      \
  };

/**
 * @brief detector_base of normal class. structure compatible with multiple data forms.
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:14
 */
struct EIGEN_ALIGN16 Normal_Base {
    ADD_UNION_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    struct {
        struct {
            float curvature;
        };
        float data_c[4];
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief The direction perpendicular to the surface at a point on the surface
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:15
 */
class Normal : public Normal_Base {
    public:
        inline explicit Normal(const Normal_Base &p) : Normal_Base() {
            normal_x = p.normal_x;
            normal_y = p.normal_y;
            normal_z = p.normal_z;
            data_n[3] = 0.0f;
            curvature = p.curvature;
        }

        inline Normal() : Normal_Base() {
            normal_x = normal_y = normal_z = data_n[3] = 0.0f;
            curvature = 0;
        }

        inline Normal(float n_x, float n_y, float n_z) : Normal_Base() {
            normal_x = n_x;
            normal_y = n_y;
            normal_z = n_z;
            curvature = 0;
            data_n[3] = 0.0f;
        }

    typedef std::shared_ptr<Normal> Ptr;
    typedef std::shared_ptr<Normal const> ConstPtr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //PERCEPTION_SOLUTION_NORMAL_HPP
