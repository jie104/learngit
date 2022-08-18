/**
 * @file path_checker.cpp
 *
 * @author pengjiali
 * @date 19-7-2.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "path_checker.h"
#include "core/exec_error.hpp"
#include "core/map/mark/Mark.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/util/utils.h"

using namespace sros::core;

namespace sros {

void sros::PathChecker::checkMoveFollowPathArgsChecker(const sros::core::NavigationPath_vector &paths) {
    // 后期可以写内层
    checkIfExistARCPath(paths);

    checkIfStartPoseOffset(paths);

    checkIfStartAngleOffset(paths);

    checkIfNotPoseContinuous(paths);

    checkIfNotAngleContinuous(paths);

    checkDisableRotateArea(paths);

    checkChassisType(paths);
}

void PathChecker::checkIfExistARCPath(const sros::core::NavigationPath_vector &paths) {
    auto &s = sros::core::Settings::getInstance();
    auto manual_path_enable_arc_path = s.getValue<std::string>("main.manual_path_enable_arc_path", "False") == "True";
    if (manual_path_enable_arc_path) {
        return;
    }

    for (auto path : paths) {
        if (path.type_ == PATH_ARC) {
            throw EXEC_ERROR(ERROR_CODE_MOVEMENT_FOLLOW_PATH_EXIST_ARC, "");
        }
    }
}

void PathChecker::checkIfStartPoseOffset(const sros::core::NavigationPath_vector &paths) {
    for (auto path : paths) {
        if (path.type_ != PATH_ROTATE) {  // 找到第一条费旋转路径
            Pose pose = src_sdk->getCurPose();
            double distance = get2PointDistance(path.sx_, path.sy_, pose.x(), pose.y());
            auto &s = sros::core::Settings::getInstance();
            auto manual_path_start_pose_check_threshold =
                s.getValue<double>("main.manual_path_start_pose_check_threshold", 100) / 1000.0;
            if (distance > manual_path_start_pose_check_threshold) {
                LOGGER(WARNING, CMD_HANDER)
                    << "manual path start pose offset: " << distance << "m, start pose: (" << path.sx_ << ", "
                    << path.sy_ << "), src pose: " << pose << " threshold: " << manual_path_start_pose_check_threshold;
                throw EXEC_ERROR(ERROR_CODE_MOVEMENT_FOLLOW_PATH_START_POSE_OFFSET, "");
            }
            break;  // 若第一条非旋转路径没问题，就没问题
        }
    }
}

void PathChecker::checkIfStartAngleOffset(const sros::core::NavigationPath_vector &paths) {
    auto path = paths.front();        // 只需要考虑起点
    if (path.type_ != PATH_ROTATE) {  // 若是旋转路径就不需要考虑了
        Pose pose = src_sdk->getCurPose();
        auto start_facing = path.s_facing_;
        auto &s = sros::core::Settings::getInstance();
        auto manual_path_start_angle_check_threshold =
            s.getValue<double>("main.manual_path_start_angle_check_threshold", 50) / 10.0 * DEGREE_TO_RAD;  // 弧度
        auto offset = std::abs(normalizeYaw(start_facing - pose.yaw()));
        if (offset > manual_path_start_angle_check_threshold) {
            LOGGER(WARNING, CMD_HANDER) << "manual path start angle offset: " << offset
                                        << ", start angle: " << start_facing << "src angle: " << pose.yaw()
                                        << " threshold: " << manual_path_start_angle_check_threshold;
            throw EXEC_ERROR(ERROR_CODE_MOVEMENT_FOLLOW_PATH_START_ANGLE_OFFSET, "");
        }
    }
}

void PathChecker::checkIfNotPoseContinuous(const sros::core::NavigationPath_vector &paths) {
    auto &s = sros::core::Settings::getInstance();
    auto manual_path_pose_continuous_check_threshold =
        s.getValue<double>("main.manual_path_pose_continuous_check_threshold", 10) / 1000.0;
    if (paths.size() < 2) {
        return;
    }
    for (auto p1_it = paths.cbegin(), p2_it = paths.cbegin() + 1; p2_it != paths.cend(); ++p2_it) {
        if (p2_it->type_ != PATH_ROTATE) {
            if (p1_it->type_ != PATH_ROTATE) {  // 排除p1指向了第一条旋转路径，而p2指向了一条非旋转路径的情况
                double distance = get2PointDistance(p1_it->ex_, p1_it->ey_, p2_it->sx_, p2_it->sy_);
                if (distance > manual_path_pose_continuous_check_threshold) {
                    LOGGER(WARNING, CMD_HANDER)
                        << "The " << std::distance(paths.cbegin(), p1_it) << "th and "
                        << std::distance(paths.cbegin(), p2_it) << "th manual path pose continuous offset: " << distance
                        << "m, end: (" << p1_it->ex_ << ", " << p1_it->ey_ << "), start: (" << p2_it->sx_ << ", "
                        << p2_it->sy_ << ") threshold: " << manual_path_pose_continuous_check_threshold;
                    throw EXEC_ERROR(ERROR_CODE_MOVEMENT_FOLLOW_PATH_POSE_NOT_CONTINUOUS, "");
                }
            }
            p1_it = p2_it;
        }
    }
}
void PathChecker::checkIfNotAngleContinuous(const sros::core::NavigationPath_vector &paths) {
    auto &s = sros::core::Settings::getInstance();
    auto manual_path_angle_continuous_check_threshold =
        s.getValue<double>("main.manual_path_angle_continuous_check_threshold", 30) / 10.0 * DEGREE_TO_RAD;  // 弧度
    for (auto it = paths.cbegin() + 1; it != paths.cend(); ++it) {
        if (it->type_ != PATH_ROTATE) {
            double offset = 0.0;  // 角度偏差
            auto it_last = it - 1;
            if (it_last->type_ == PATH_ROTATE) {
                auto start_facing = it->s_facing_;
                offset = std::abs(normalizeYaw(start_facing - it_last->rotate_angle_));
            } else {
                offset = minRotate(it->s_facing_, it_last->e_facing_);
            }

            if (offset > manual_path_angle_continuous_check_threshold) {
                LOGGER(WARNING, CMD_HANDER)
                    << "The " << std::distance(paths.cbegin(), it) << "th and "
                    << std::distance(paths.cbegin(), it_last) << "th manual path angle continuous offset: " << offset
                    << " threshold: " << manual_path_angle_continuous_check_threshold;
                throw EXEC_ERROR(ERROR_CODE_MOVEMENT_FOLLOW_PATH_ANGLE_NOT_CONTINUOUS, "");
            }
        }
    }
}

void PathChecker::checkDisableRotateArea(const sros::core::NavigationPath_vector &paths) {
    auto rotate_pose = src_sdk->getCurPose();  // 旋转是在哪个点发生的
    for (auto it = paths.begin(); it != paths.end(); ++it) {
        if (it->type_ == PATH_ROTATE) {
            auto areas = MapManager::getInstance()->getInsideArea(rotate_pose.x() * 100, rotate_pose.y() * 100,
                                                                  map::AreaMark::AREA_TYPE_DISABLE_ROTATE);
            for (const auto &area : areas) {
                double threshold_angle = area.param_int / 1000.0;
                double rotate_angle = std::abs(normalizeYaw(it->rotate_angle_ - rotate_pose.yaw()));
                LOG(INFO) << "check disable rotate area " << area.id << ", threshold " << threshold_angle
                          << ". rotate angle is " << rotate_angle << ", rotate_pose " << rotate_pose;
                if (threshold_angle < rotate_angle) {
                    std::string msg = "area " + std::to_string(area.id) + " 's rotate threshold is " +
                                      std::to_string(threshold_angle) + ", but path " +
                                      std::to_string(std::distance(paths.begin(), it) + 1) + " rotate is " +
                                      std::to_string(rotate_angle);
                    throw EXEC_ERROR(ERROR_CODE_MOVEMENT_ROTATE_IN_DISABLE_ROTATE_AREA, msg);
                }
            }
        } else {  // 更新旋转的位置
            rotate_pose.x() = it->ex_;
            rotate_pose.y() = it->ey_;
            rotate_pose.yaw() = it->e_facing_;
        }
    }
}

void PathChecker::checkChassisType(const NavigationPath_vector &paths) {
    // 非万向底盘不允许路径车头朝向设置为左和右
    if (src_sdk->getBaseType() != sdk::DUAL_STR && src_sdk->getBaseType() != sdk::DIFF_TWIN_BASE) {
        for (const auto &path : paths) {
            if (path.direction_ == PathDirection::PATH_LEFT || path.direction_ == PathDirection::PATH_RIGHT) {
                throw EXEC_ERROR(ERROR_CODE_MOVEMENT_PATHS_CHASSIS_NOT_SUPPORT,
                                 "Path direction value LEFT and RIGHT only support DUAL_STR.");
            }
        }
    }
}
}  // namespace sros
