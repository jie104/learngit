//
// Created by lhx on 18-6-2.
//

#ifndef SROS_OBSTACLE_FINDER_H
#define SROS_OBSTACLE_FINDER_H

#include <opencv2/core/core.hpp>

class ObstacleFinder {
public:
    ObstacleFinder();
    ~ObstacleFinder() = default;

    void enableDebugOutput(bool is_enable);

    // 设置参数并重新计算其他参数
    void setParameter(int ob_area_width, int ob_area_length, int ob_area_height,
                      int ob_area_min, int ob_area_max,
                      double camera_install_pitch, int camera_install_height,
                      double camera_fov_width, double camera_fov_height,
                      int camera_depth_image_width, int camera_depth_image_height);

    // 根据设置的参数更新其他参数
    void updateParameter();

    std::vector<cv::Rect> processImage(const cv::Mat & input,
                                       std::vector<std::vector<cv::Point>> &ob_points);

    double calculateFloorDepth(const cv::Point &p) const;

    // 判断点p是否在避障区域内
    bool is_inside(const cv::Point2i &p) const;

    int A_W = 600; // 避障区域
    int A_H = 3000;
    int A_Z = 500;

    double ALPHA = 10.8; // 安装倾角，单位度
    int h = 250; // 安装高度，单位mm

    int H = 480; // 深度图大小
    int W = 848;

    double BETA = 58.0 / 2; // FOV * 1/2
    double GAMA = 85.2 / 2; // FOV * 1/2

    int OBSTACLE_AREA_MIN = 18;
    int OBSTACLE_AREA_MAX = 10000;

    const double RAD_TO_DEGREE = 180 / M_PI;
    const double DEGREE_TO_RAD = M_PI / 180;

    // 地面的参考深度值
    cv::Mat floor_depth = cv::Mat::zeros(H, W, CV_16UC1);

    // 用于确定避障区域的参考点
    cv::Point2d p0, p1, p2, p3, p4, p5, p6, p7;

    double land_line_y;

    bool enable_debug_output;
};



#endif //SROS_OBSTACLE_FINDER_H
