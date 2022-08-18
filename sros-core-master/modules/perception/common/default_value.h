/**
 * @file constants.h
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/26
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SRC_DEFAULT_VALUE_H
#define SRC_DEFAULT_VALUE_H
// INCLUDE
#include <string>

// CODE
namespace perception {

/// base parameter.
    const std::string PERCEPTION_LIBRARY_NAME = "libperception";
    const std::string DEFAULT_LOG_PATH = "/home/zx/dev/log/";
    const std::string DEFAULT_DATA_PATH = "/home/zx/dev/data/";
    const std::string DEFAULT_TOPIC_NAME = "/o3d3xx/camera/cloud";
    const std::string DEFAULT_CONFIG_PATH = "/home/zx/Projects/perception/example-cpp/config/detectors_param.config";
    const bool DEFAULT_PCL_VIEWER_ENABLE = true;
    const bool DEFAULT_DATA_FROM_REALTIME = true;
    const bool DEFAULT_DATA_RECORD = false;
    const std::string DEFAULT_WORK_MODE = "debug";
    const std::string FRAME_ID_BASE = "/detecter/card";
    const float DEFAULT_CAMERA_X_OFFSET = -0.1f;
    const float DEFAULT_CAMERA_Y_OFFSET = 0.0f;
    const float DEFAULT_CAMERA_YAW = 180.0f;
    const float DEFAULT_LASER_X_OFFSET = -0.1f;
    const float DEFAULT_LASER_Y_OFFSET = 0.0f;
    const float DEFAULT_LASER_YAW = 180.0f;
    const float DEFAULT_FORK_END_X = 1.0f;
    const float DEFAULT_FORK_END_Y = 0.0f;
    const float DEFAULT_FORK_END_Z = 0.0f;
    const int DEFAULT_FORKING_ANGLE_LIMIT = 10;
    const float DEFAULT_FORKING_Y_LIMIT = 0.3f;


/// card detector parameter.
    const float CARD_DEFAULT_MIN_X = 1;
    const float CARD_DEFAULT_MAX_X = 3.5;
    const float CARD_DEFAULT_MIN_Y = -0.8;
    const float CARD_DEFAULT_MAX_Y = 0.8;
    const float CARD_DEFAULT_MIN_Z = -0.45;
    const float CARD_DEFAULT_MAX_Z = -0.38;
    const int CARD_DEFAULT_MIN_CLUSTER_SIZE = 20;
    const int CARD_DEFAULT_MAX_CLUSTER_SIZE = 300;
    const float CARD_DEFAULT_POINT_DIST_TOLERANCE = 0.03;
    const int CARD_DEFAULT_THRESHOLD = 100;
    const float CARD_DEFAULT_NORMAL_ANGLE_TOLERANCE = 60.0f;
    const float CARD_DEFAULT_BOTH_PALLET_DIST_ALLOW = 0.04f;
    const int CARD_DEFAULT_UP_EXTEND_PIXEL = 15;
    const int CARD_DEFAULT_DOWN_EXTEND_PIXEL = 10;
    const float CARD_DEFAULT_MIN_AREA_RATE = 0.7f;
    const float CARD_DEFAULT_MAX_AREA_RATE = 1.2f;
    const float CARD_DEFAULT_HOLE_HEIGHT_RATE_DIFF_ALLOW = 2.0f;
    const float CARD_DEFAULT_PALLET_NORMAL_DIFF_ANGLE_ALLOW = 5.0f;

/// cricle detector parameter.
    const float CIRCLE_DEFAULT_MIN_X = 1;
    const float CIRCLE_DEFAULT_MAX_X = 3;
    const float CIRCLE_DEFAULT_MIN_Y = -1;
    const float CIRCLE_DEFAULT_MAX_Y = 1;
    const float CIRCLE_DEFAULT_MIN_Z = -0.45;
    const float CIRCLE_DEFAULT_MAX_Z = 1;
    const int CIRCLE_DEFAULT_MIN_CLUSTER_SIZE = 3000;
    const int CIRCLE_DEFAULT_MAX_CLUSTER_SIZE = 15000;
    const float CIRCLE_DEFAULT_POINT_DIST_TOLERANCE = 0.03;
    const int CIRCLE_DEFAULT_THRESHOLD = 100;
    const float CIRCLE_DEFAULT_NORMAL_ANGLE_TOLERANCE = 60.0f;
    const int CIRCLE_DEFAULT_MAX_ITERATION = 20;
    const float CIRCLE_DEFAULT_FITTING_RATE = 0.9;
    const float CIRCLE_DEFAULT_PIXEL_DIST_ALLOW = 3;
    const int CIRCLE_DEFAULT_CONTOUR_MIN_SIZE = 100;

/// handcart detector parameter.
    const float HANDCART_DEFAULT_MIN_X = 1.0f;
    const float HANDCART_DEFAULT_MAX_X = 1.7f;
    const float HANDCART_DEFAULT_MIN_Y = -0.5f;
    const float HANDCART_DEFAULT_MAX_Y = 0.5f;
    const float HANDCART_DEFAULT_MIN_Z = -0.26f;
    const float HANDCART_DEFAULT_MAX_Z = -0.16f;
    const int HANDCART_DEFAULT_MIN_CLUSTER_SIZE = 500;
    const int HANDCART_DEFAULT_MAX_CLUSTER_SIZE = 1500;
    const float HANDCART_DEFAULT_POINT_DIST_TOLERANCE = 0.03;
    const float HANDCART_DEFAULT_RATE_THRESHOLD = 1.2f;
    const float HANDCART_DEFAULT_NORMAL_ANGLE_TOLERANCE = 60.0f;
    const float HANDCART_DEFAULT_ACROSS_DIST_ALLOW = 0.04f;

/// decards detector parameter.
    const float DECARDS_DEFAULT_MIN_X = 1.5;
    const float DECARDS_DEFAULT_MAX_X = 3.0;
    const float DECARDS_DEFAULT_MIN_Y = -0.8;
    const float DECARDS_DEFAULT_MAX_Y = 0.8;
    const float DECARDS_DEFAULT_MIN_Z = -0.5;
    const float DECARDS_DEFAULT_MAX_Z = 3.0;
    const int DECARDS_DEFAULT_THRESHOLD = 60;
    const int DECARDS_DEFAULT_UPPER_LIP_HEIGHT = 5;
    const int DECARDS_DEFAULT_STACK_TOP_MARGIN = 50;
    const int DECARDS_DEFAULT_STACK_MIN_AREA = 200;
    const int DECARDS_DEFAULT_STACK_MAX_AREA = 600000;
    const int DECARDS_DEFAULT_TARGET_POINTS_MIN_COUNT = 100;
    const int DECARDS_DEFAULT_TARGET_POINTS_MAX_COUNT = 800;
    const float DECARDS_DEFAULT_TARGET_MIN_WIDTH_RADIO = 0.8f;
    const float DECARDS_DEFAULT_TARGET_MAX_WIDTH_RADIO = 1.2f;
    const int DECARDS_DEFAULT_STACK_MIN_COL = 100;
    const int DECARDS_DEFAULT_STACK_MAX_COL = 200;
    const float DECARDS_DEFAULT_STACK_MIN_RECTANGULARITY = 0.5f;
    const float DECARDS_DEFAULT_STACK_MAX_RECTANGULARITY = 1.0f;

    /// putspace detector parameter
    const int PUTSPACE_HEADUP_DEFAULT_MIN_CLOUD_SIZE         = 2000;
    const int PUTSPACE_HEADUP_DEFAULT_MAX_CLOUD_SIZE         = 20000;
    const float PUTSPACE_HEADUP_DEFAULT_FRONT_NORMAL_ANGLE_TOLERANCE = 60.0f;
    const float PUTSPACE_HEADUP_DEFAULT_TOP_NORMAL_ANGLE_TOLERANCE   = 60.0f;

}
#endif //SRC_DEFAULT_VALUE_H
