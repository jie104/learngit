/**
 * @file point3d.hpp
 * @brief point of 3d-coordinate system.
 *
 * point of 3d-coordinate system. Overloaded the (=,-,==) sign operator。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_POINT3D_HPP
#define PERCEPTION_SOLUTION_POINT3D_HPP

// INCLUDE
#include <Eigen/Dense>
#include <memory>

// CODE
#define ADD_UNION_POINT3D    \
  union EIGEN_ALIGN16 {      \
    float data[3];           \
    struct {                 \
      float x;               \
      float y;               \
      float z;               \
    };                       \
  };

#define ADD_EIGEN_MAPS_POINT3D \
  inline Eigen::Map<Eigen::Vector3f> point() { return (Eigen::Vector3f::Map(data));  } \
  inline const Eigen::Map<const Eigen::Vector3f> point() const { return (Eigen::Vector3f::Map(data)); } \

#ifndef EPSILON
    #define EPSILON                 0.00001
#endif //EPSILON

/**
 * @brief point of 3d-coordinate system.
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:12
 */
struct Point3D {
public:
    ADD_UNION_POINT3D

    ADD_EIGEN_MAPS_POINT3D

    /**
     * @brief default Constructor
     */
    Point3D():x(0),y(0),z(0){};

    /**
     * @brief Constructor overload with x,y,z parameters
     * @param[in] x X coordinate
     * @param[in] y Y coordinate
     * @param[in] z Z coordinate
     */
    Point3D(const float &x, const float &y, const float &z) : x(x), y(y), z(z) {}

    /**
     * @brief Constructor overload with point[4] parameters
     * @param[in] p[3] point(x,y,z)
     */
    Point3D(const float p[]) : x(p[0]), y(p[1]), z(p[2]) {}

    /**
     * @brief reset point x=0 y=0 z=0.
     */
    inline void reset() {
        this->x = this->y = this->z = .0f;
    }

    /**
     * @brief The minus operator override returns the distance between the
     *        point and this point
     * @param[in] point x, y
     * @return the distance between the point and this point
     */
    float operator-(const Point3D &point) const {
        return sqrtf((this->x - point.x) * (this->x - point.x)
                     + (this->y - point.y) * (this->y - point.y)
                     + (this->z - point.z) * (this->z - point.z));
    }

    /**
     * @brief Assignment constructor
     * @param[in] point input point
     * @return Object after assignment
     */
    Point3D& operator=(const Point3D &point) {
        this->x = point.x;
        this->y = point.y;
        this->z = point.z;
        return *this;
    }

    /**
     * @brief Judge whether two points are the same point. If they are the same point,
     *        return true, otherwise return false.
     * @param[in] point
     * @return
     */
    bool operator==(const Point3D &point) const {
        if ((this->x - point.x) < EPSILON
            && (this->y - point.y) < EPSILON) {
            return true;
        }
        return false;
    }

    typedef std::shared_ptr<Point3D> Ptr;
    typedef std::shared_ptr<Point3D const> ConstPtr;
};

using Point3DPtr = Point3D::Ptr;
using Point3DConstPtr = Point3D::ConstPtr;

/**
 * @brief : Camera position relative to AGV.
 */
struct Pose3D{
    /** @brief Angle of rotation in the X,Y,Z direction of the camera relative to the AGV. (Unit: degree) */
    Point3D rotation;

    /** @brief The coordinate of the camera in the AGV coordinate system. (Unit: m) */
    Point3D coordinate;

    typedef std::shared_ptr<Pose3D> Ptr;
    typedef std::shared_ptr<Pose3D const> ConstPtr;
};

using Pose3DPtr = Point3D::Ptr;
using Pose3DConstPtr = Point3D::ConstPtr;

#endif //PERCEPTION_SOLUTION_POINT3D_HPP
