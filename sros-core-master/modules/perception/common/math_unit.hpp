//
// Created by zx on 2020/12/9.
//

#ifndef PERCEPTION_SOLUTION_MATH_UNIT_HPP
#define PERCEPTION_SOLUTION_MATH_UNIT_HPP

#include <math.h>

#include "normal.hpp"
#include "point_cloud.hpp"

/**
    * @brief euclidean distance from current point to origin
    * @param[in] x X coordinate of point
    * @param[in] y Y coordinate of point
    * @param[in] z Z coordinate of point
    * @return return distance by float type.
    */
static float distancePointXYZ(const float &x, const float &y, const float &z) {
    return sqrt(x * x + y * y + z * z);
}

/**
    * @brief Calculate the angle between two normal vectors.
    * @param[in] n1 normal one.
    * @param[in] n2 normal two.
    * @return angle of the both normal.
    */
static float
angleBothVector(const Normal &n1,
                const Normal &n2 ) {
    float x1 = n1.normal_x, y1 = n1.normal_y, z1 = n1.normal_z;
    float x2 = n2.normal_x, y2 = n2.normal_y, z2 = n2.normal_z;
    float across = x1 * x2 + y1 * y2 + z1 * z2;
    float mod = sqrtf(x1 * x1 + y1 * y1 + z1 * z1) * sqrtf(x2 * x2 + y2 * y2 + z2 * z2);

    if (mod < EPSILON) return 0;

    float cos_theta = fabsf(across / mod);
    return acos(cos_theta) * 180 / M_PI;
}

/**
 * @brief Calculate the Euclidean distance between two 2d-points.
 * @param[in] x1 point one x.
 * @param[in] y1 point one y.
 * @param[in] x2 point two x.
 * @param[in] y2 point two y.
 * @return Euclidean distance between two points.
 */
static float
distanceBothPoint2d(const float &x1, const float &y1,
                    const float &x2, const float &y2) {
    return sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * @brief Calculate the Euclidean distance between two 3d-points.
 * @param[in] a point a(x,y,z)
 * @param[in] b point b(x,y,z)
 * @return
 */
static float
distanceBothPoint3d(const Point3D &a,
                    const Point3D &b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}



#endif //PERCEPTION_SOLUTION_MATH_UNIT_HPP
