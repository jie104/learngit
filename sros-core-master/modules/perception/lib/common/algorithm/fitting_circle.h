/**
 * @file fitting_circle.h
 * @brief ransac fitting circle
 * 
 * SACSegmentation represents the Nodelet segmentation class for
 * Sample Consensus methods and models, in the sense that it just creates a
 * Nodelet wrapper for generic-purpose SAC-based segmentation.
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/19
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_FITTING_CIRCLE_H
#define PERCEPTION_SOLUTION_FITTING_CIRCLE_H

// INCLUDE
#include<vector>
#include"../../../common/point2d.hpp"

//CODE
/** \brief SACSegmentation represents the Nodelet segmentation class for
 * Sample Consensus methods and models, in the sense that it just creates a
 * Nodelet wrapper for generic-purpose SAC-based segmentation.
 */
class FittingCircle {
public:
    /** \brief default constructor. */
    FittingCircle()  = default;

    /** \brief default deconstruction. */
    ~FittingCircle() = default;

    /** \brief Provide a pointer to the input dataset
     * \param[in] points the const std shared pointer to a Point message
     */
    inline void setInputCloud(const std::vector<Point2D> &points) {
        this->points_ = points;
    }

    /**
     * \brief  Set the maximum allowable distance from the interior point to the model
     * \param[in] distance_threshold distance value.
     */
    inline void setDistanceThreshold(const float distance_threshold) {
        this->distance_threshold_ = distance_threshold;
    }

    /** \brief The registration fails if the number of iterations exceeds the maximum number of iterations.
     * \param[in] max_iterations maximum number of iterations */
    inline void setMaxIterations(const int max_iterations) {
        this->max_iterations_ = max_iterations;
    }

    /**
     * \brief Set the fit point ratio.
     * \param[in] rate rate value.
     */
    inline void setPointRate(const float rate){
        this->point_rate_ = rate;
    }

    /**
     * \brief Fitting plane by ransac method.
     * \param[out] indices.
     * \param[out] coefficients_plane.
     */
    void fitting(
        std::vector<int> &indices,
        std::vector<float> &coefficients_circle);

private:

    /**
     * @brief Fitting circular equation from a set of points by least square method.
     * @param[in] points point set.
     * @param[in] indices index of point in point set.
     * @param[out] center_x circle center x.
     * @param[out] center_y circle center y.
     * @param[out] radius circle radius.
     * @return if fitting success, return true, otherwise return false.
     */
    bool circleLeastFit(const std::vector<Point2D> &points,
                        const std::vector<int> &indices,
                        double &center_x,
                        double &center_y,
                        double &radius);

    /**
     * @brief get set of rand num.
     * @param[out] nums
     */
    inline void getRandNum(std::vector<int> &nums);

    /** \brief If the percentage of points is greater than rate, the iteration is terminated. */
    float point_rate_;

    /** \brief Maximum allowable distance of interior point. */
    float distance_threshold_;

    /** \brief The input point dataset. */
    std::vector<Point2D> points_;

    /** \brief Maximum number of iterations before giving up (user given parameter). */
    int max_iterations_;
};


#endif //PERCEPTION_SOLUTION_FITTING_CIRCLE_H
