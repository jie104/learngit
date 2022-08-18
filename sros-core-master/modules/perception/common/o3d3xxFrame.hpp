/**
 * @file o3dxxFrame.hpp
 * @brief sensor data of 03d303
 *
 * sensor data of 03d303, Inherit to camera3D detector_base class, include xyz-image, amplitude image and so on.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef PERCEPTION_SOLUTION_O3D3XXFRAME_HPP
#define PERCEPTION_SOLUTION_O3D3XXFRAME_HPP

// INCLUDE
#include "camera3dFrame.hpp"

/**
 * @brief sensor data of 03d303
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:17
 */
struct O3d3xxFrame : Camera3DFrame
{

    /** @brief image cell point(x,y,z). */
    cv::Mat xyz_img;

    /** @brief shows how much a point is beating. */
    cv::Mat_<uint16_t> amplitude_img;

    /** @brief Original amplitude image. */
    cv::Mat_<uint16_t> raw_amplitude_img;

    /** @brief For good and bad images, 1 is good and 0 is bad. */
    cv::Mat_<uint8_t> good_bad_pixels_img;

    /**
     * @brief Assignment operator overloading.
     * @param[in] frame input source data.
     * @return the object after deep copy.
     */
    O3d3xxFrame & operator = (const O3d3xxFrame &frame) {
        this->time_point = frame.time_point;
        this->cloud->points.resize(frame.cloud->size());
        copy(frame.cloud->points.begin(), frame.cloud->points.end(), this->cloud->points.begin());
        this->cloud->image_indices.resize(this->cloud->size());
        for (size_t i = 0; i < frame.cloud->size(); ++i){
            this->cloud->image_indices[i] = i;
        }
        frame.confidence_img.copyTo(this->confidence_img);
        frame.distance_img.copyTo(this->distance_img);
        frame.amplitude_img.copyTo(this->amplitude_img);
        frame.raw_amplitude_img.copyTo(this->raw_amplitude_img);
        frame.xyz_img.copyTo(this->xyz_img);

        return *this;
    }

    /**
     * @brief clear the runtime data.
     */
    void reset() override{
        this->cloud->points.clear();
        this->confidence_img.setTo(0);
        this->distance_img.setTo(0);
        this->xyz_img.setTo(0);
        this->amplitude_img.setTo(0);
        this->raw_amplitude_img.setTo(0);
        this->good_bad_pixels_img.setTo(0);
    }
};
#endif //PERCEPTION_SOLUTION_O3D3XXFRAME_HPP
