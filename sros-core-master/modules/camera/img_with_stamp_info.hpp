//
// Created by lfc on 19-10-26.
//

#ifndef SROS_IMG_WITH_TIME_MSG_HPP
#define SROS_IMG_WITH_TIME_MSG_HPP

#include <opencv/cv.h>
#include <memory>
#include "core/msg/ObstacleMsg.hpp"

namespace camera {
struct ImgWithStampInfo {
    std::string topic_name;
    std::string camera_name;
    std::string points_topic_name;
    int64_t stamp;
    cv::Mat img;
    cv::Mat xyz_img;
    cv::Mat amplitude_img;  // add by zhangxu at 2020/11/30
    float fork_height_encoder; // add by lijunhong at 2021/12/13
    sros::core::ObstacleMsg_ptr points;
    bool use_mat_info = true;
    bool use_points_info = false;
};

typedef std::shared_ptr<ImgWithStampInfo> ImgWithStampInfoPtr;
}  // namespace camera

#endif  // SROS_IMG_WITH_TIME_MSG_HPP
