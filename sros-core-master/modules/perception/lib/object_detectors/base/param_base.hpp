/**
 * @file detect_param.h
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/16
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __DETECT_PARAM_H__
#define __DETECT_PARAM_H__
//INCLUDE
#include "../../../common/point3d.hpp"
#include <Eigen/Dense>

//CODE
/**
 * @brief : A cube region in space.
 */
template <class T=float>
struct Range3D{
    /** @brief pass through filter point.x > min_x. (Unit: m) */
    T min_x;

    /** @brief pass through filter point.x < max_x. (Unit: m) */
    T max_x;

    /** @brief pass through filter point.y > min_y. (Unit: m) */
    T min_y;

    /** @brief pass through filter point.y < max_y. (Unit: m) */
    T max_y;

    /** @brief pass through filter point.z > min_z. (Unit: m) */
    T min_z;

    /** @brief pass through filter point.z < max_z. (Unit: m) */
    T max_z;
};

/**
 * @brief
 */
struct ParamBase {

    /** @brief detect rangle x=[min_x,max_x] y=[min_y, max_y] z=[min_z,max_z] */
    Range3D<float> detect_range{};

    /** @brief Installation position of camera relative to AGV */
    Eigen::Vector3f camera_fixed{};

    /** @brief Installation position of laser relative to AGV */
    Eigen::Vector3f laser_fixed{};

    /** @brief The translation of the fork-end in the XYZ direction relative to the AGV. (Unit: m) */
    Eigen::Vector3f fork_end{};

    // 目标栈板的高度
    float target_value_z_ = 0.;
    //相机沿Z轴放到地面时的安装偏移量
    Eigen::Vector3f camera_to_agv_transf{};
    //相机沿Z轴放到地面时的旋转偏移
    Eigen::Vector3f camera_to_agv_rotate{};  // roll, pitch, yaw   

    typedef  std::shared_ptr<ParamBase> Ptr;
    typedef  std::shared_ptr<const ParamBase> ConstPtr;
};

using DetectParamPtr = ParamBase::Ptr;
using DetectParamConstPtr = ParamBase::ConstPtr;

#endif //PERCEPTION_SOLUTION_DETECT_PARAM_H