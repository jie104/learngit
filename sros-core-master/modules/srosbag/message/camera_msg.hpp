//
// Created by duan on 2021/6/16.
//

#ifndef SROS_CAMERA_MSG_H
#define SROS_CAMERA_MSG_H

#include "std_msg.hpp"
#include <opencv2/opencv.hpp>
#include <memory>

namespace camera_msg{
struct Camera{
    std_msgs::Header header;
    cv::Mat left_rgb;           //CV_8U
    cv::Mat right_rgb;          //CV_8U
    cv::Mat depth;              //CV_16S

    using Ptr = std::shared_ptr<Camera>;
    int status{0};   // 0:未完成  1:正常数据  2:数据出现异常根据异常前数据推导
    static std::string msgType(){return "cameraBag";}
    static bool isCompress() {return false;}
};

inline bool operator<(const Camera::Ptr &lhs, const Camera::Ptr &rhs){
    return lhs->header.stamp < rhs->header.stamp;
}

inline bool operator>(const Camera::Ptr &lhs, const Camera::Ptr &rhs){
    return lhs->header.stamp > rhs->header.stamp;
}

inline bool operator<(const Camera &lhs, const Camera &rhs){
    return lhs.header.stamp < rhs.header.stamp;
}

inline bool operator>(const Camera &lhs, const Camera &rhs){
    return lhs.header.stamp > rhs.header.stamp;
}

}

#endif  // SROS_CAMERA_MSG_H
