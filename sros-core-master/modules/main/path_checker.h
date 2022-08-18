/**
 * @file path_checker.h
 *
 * @author pengjiali
 * @date 19-7-2.
 *
 * @describe 检测move_follow_path的输入路劲
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MODULES_MAIN_MOVE_FOLLOW_PATH_ARGS_CHECKER_H_
#define MODULES_MAIN_MOVE_FOLLOW_PATH_ARGS_CHECKER_H_

#include <utility>
#include "core/navigation_path.h"
#include "core/state.h"

namespace sros {

class PathChecker {
 public:
    static void checkMoveFollowPathArgsChecker(const sros::core::NavigationPath_vector &paths); // 会抛出异常

    static void checkIfExistARCPath(const sros::core::NavigationPath_vector &paths);  // 检测是否存在圆弧路径

    static void checkIfStartPoseOffset(const sros::core::NavigationPath_vector &paths);  // 检测路径的起始点与agv当前位置偏差是否过大

    static void checkIfStartAngleOffset(const sros::core::NavigationPath_vector &paths);  // 检测路径的起始点与agv当前角度的偏差是否过大

    static void checkIfNotPoseContinuous(const sros::core::NavigationPath_vector &paths);  // 检测路径路径位置是否不连续

    static void checkIfNotAngleContinuous(const sros::core::NavigationPath_vector &paths);  // 检测路径角度是否不连续

    static void checkDisableRotateArea(const sros::core::NavigationPath_vector &paths); // 检测路径是否在禁止旋转区域旋转

    static void checkChassisType(const sros::core::NavigationPath_vector &paths); // 检查路径是否适合当前底盘
};

}  // namespace sros

#endif  // MODULES_MAIN_MOVE_FOLLOW_PATH_ARGS_CHECKER_H_
