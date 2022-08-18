/**
 * @file camera3d.hpp
 * @brief detector_base of camera3d
 *
 * sensor of 3d camera detector_base data, example timestamp, pointcloud, confident image, deep image.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_CAMERA3DFRAME_HPP
#define PERCEPTION_SOLUTION_CAMERA3DFRAME_HPP

// INCLUDE
#include <chrono>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "point_cloud.hpp"

/**
 * @brief detector_base of camera3d
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:19
 */
struct Camera3DFrame
{
    Camera3DFrame():cloud(std::make_shared<PointCloud>()){}

    /**
     * @brief clear runtime data.
     */
    virtual void reset(){
        this->cloud->points.clear();
        this->confidence_img.setTo(0);
        this->distance_img.setTo(0);
    }

    /** @brief The point in time when the frame is acquired */
    uint64_t time_point;

    /** @brief camera 3d points */
    PointCloud::Ptr cloud;

    /** @brief Indicates the quality of a point */
    cv::Mat_<uint8_t> confidence_img;

    /** @brief the distance from the point to the camera */
    cv::Mat_<uint16_t> distance_img;
};

#endif //PERCEPTION_SOLUTION_CAMERA3DFRAME_HPP
