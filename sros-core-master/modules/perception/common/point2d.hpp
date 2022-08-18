/**
 * @file point2D.hpp
 * @brief point of 2d-coordinate system.
 *
 * point of 2d-coordinate system. Overloaded the (=,-,==) sign operator。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_POINT2D_HPP
#define PERCEPTION_SOLUTION_POINT2D_HPP

// INCLUDE
#include <Eigen/Dense>
#include <memory>

// CODE
#define ADD_UNION_POINT2D    \
  union EIGEN_ALIGN16 {      \
    float data[2];           \
    struct {                 \
      float x;               \
      float y;               \
    };                       \
  };

#define ADD_EIGEN_MAPS_POINT2D \
  inline Eigen::Map<Eigen::Vector2f> point() { return (Eigen::Vector2f::Map(data));  } \
  inline const Eigen::Map<const Eigen::Vector2f> point() const { return (Eigen::Vector2f::Map(data)); } \

#ifndef EPSILON
#define EPSILON                 0.00001
#endif //EPSILON

/**
 * @brief point of 2d-coordinate system.
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:12
 */
struct Point2D {
public:
    ADD_UNION_POINT2D

    ADD_EIGEN_MAPS_POINT2D

    /**
     * @brief default Constructor
     */
    Point2D() = default;

    /**
     * @brief Constructor overload with x,y,z parameters
     * @param[in] x X coordinate
     * @param[in] y Y coordinate
     * @param[in] z Z coordinate
     */
    Point2D(const float &x, const float &y) : x(x), y(y) {}

    /**
     * @brief Constructor overload with point[4] parameters
     * @param[in] p[3] point(x,y,z)
     */
    Point2D(const float p[]) : x(p[0]), y(p[1]) {}

    /**
     * @brief reset point x=0 y=0 z=0.
     */
    inline void reset() {
        this->x = this->y = .0f;
    }

    /**
     * @brief The minus operator override returns the distance between the
     *        point and this point
     * @param[in] point x, y
     * @return the distance between the point and this point
     */
    float operator-(const Point2D &point) const {
        return sqrtf((this->x - point.x) * (this->x - point.x)
                     + (this->y - point.y) * (this->y - point.y));
    }

    /**
     * @brief Assignment constructor
     * @param[in] point input point
     * @return Object after assignment
     */
    Point2D& operator=(const Point2D &point) {
        this->x = point.x;
        this->y = point.y;
        return *this;
    }

    /**
     * @brief Judge whether two points are the same point. If they are the same point,
     *        return true, otherwise return false.
     * @param[in] point
     * @return
     */
    bool operator==(const Point2D &point) const {
        if ((this->x - point.x) < EPSILON
            && (this->y - point.y) < EPSILON) {
            return true;
        }
        return false;
    }

    typedef std::shared_ptr<Point2D> Ptr;
    typedef std::shared_ptr<Point2D const> ConstPtr;
};

using Point2DPtr = Point2D::Ptr;

#endif //PERCEPTION_SOLUTION_POINT2D_HPP
