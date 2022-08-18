/**
 * @file circle_target.h
 * @brief Circular target class
 * 
 * Target class for detecting circular targets
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/16
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CIRCLE_TARGET_H__
#define __CIRCLE_TARGET_H__

// INCLUDE
#include "../base/target.hpp"
#include <opencv/cv.h>

//CODE
namespace perception {
/**
 * @description : Circular target class
 * @author      : zhangxu
 * @date        : 2020/12/16 上午11:24
 */
class CircleTarget : public Target {
public:

    bool operator<(const CircleTarget& circle) const {
        return this->score_ < circle.score_; //大顶堆
    }

    CircleTarget() : Target()
                     , radius_(.0f)
                     , radius_pixel_(0)
                     , score_(0) {
                         // to do nothing;
                     };

    CircleTarget& operator=(const CircleTarget& circle_target) {
        // parent part.
        this->id_ = circle_target.id_;
        this->angle_ = circle_target.angle_;
        this->pose_ = circle_target.pose_;
        this->pose_normal_ = circle_target.pose_normal_;
        copyPointCloud(circle_target.point_cloud_, this->point_cloud_);
        copyNormalCloud(circle_target.normal_cloud_, this->normal_cloud_);

        // own part.
        this->score_ = circle_target.score_;
        this->radius_ = circle_target.radius_;
        this->radius_pixel_ = circle_target.radius_pixel_;
        this->center_in_cloud_ = circle_target.center_in_cloud_;
        this->center_in_image_ = circle_target.center_in_image_;
        this->contour_points_.resize(circle_target.contour_points_.size());
        copy(circle_target.contour_points_.begin(), circle_target.contour_points_.end(),
             this->contour_points_.begin());
        this->contour_img_ = circle_target.contour_img_.clone();

//        LOG(INFO) << "circle target operator=!";
        return *this;
    }

    CircleTarget(const CircleTarget &circle_target) : Target(circle_target) {
        this->score_ = circle_target.score_;
        this->radius_ = circle_target.radius_;
        this->radius_pixel_ = circle_target.radius_pixel_;
        this->center_in_cloud_ = circle_target.center_in_cloud_;
        this->center_in_image_ = circle_target.center_in_image_;
        this->contour_points_.resize(circle_target.contour_points_.size());
        copy(circle_target.contour_points_.begin(), circle_target.contour_points_.end(),
             this->contour_points_.begin());
        this->contour_img_ = circle_target.contour_img_.clone();
//        LOG(INFO) << "copy circle target constructor!";
    }

    void reset() override;

    ~CircleTarget() override = default;

    inline float getRadius() const {
        return this->radius_;
    }

    inline void setRadius(const float radius) {
        this->radius_ = radius;
    }

    inline int getRadiusPixel() const {
        return this->radius_pixel_;
    }

    inline void setRadiusPixel(const int radius_pixel) {
        this->radius_pixel_ = radius_pixel;
    }

    inline cv::Point getCenterInImage() const {
        return this->center_in_image_;
    }

    inline void setCenterInImage(const cv::Point &center) {
        this->center_in_image_ = center;
    }

    inline Point3D getCenterInCloud() const {
        return this->center_in_cloud_;
    }

    inline void setCenterInCloud(const Point3D &center_point3d) {
        this->center_in_cloud_ = center_point3d;
    }

    inline void setScore(float score) {
        this->score_ = score;
    }

    inline float getScore() const {
        return this->score_;
    }

    inline void setContourImage(const cv::Mat& img){
        this->contour_img_ = img.clone();
    }

    inline cv::Mat getContourImage() const {
        return this->contour_img_;
    }

    inline std::vector<cv::Point>& getContourPoint() const {
        return (std::vector<cv::Point> &)this->contour_points_;
    }

    inline void setContourPoint(const std::vector<cv::Point>& points){
        this->contour_points_.resize(points.size());
        copy(points.begin(), points.end(), this->contour_points_.begin());
    }

private:
    float score_;

    cv::Mat contour_img_;

    std::vector<cv::Point> contour_points_;

    float radius_;

    int radius_pixel_;

    cv::Point center_in_image_;

    Point3D center_in_cloud_{};
};

} // end of namespace perception
#endif //PERCEPTION_SOLUTION_CIRCLE_TARGET_H
