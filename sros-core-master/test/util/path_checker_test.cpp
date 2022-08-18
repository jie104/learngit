/**
 * @file path_checker_test
 *
 * @author pengjiali
 * @date 2021/4/1.
 *
 * @describe
 *
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include "../../core/settings.h"
#include "../../modules/main/path_checker.h"

using namespace sros::core;
using namespace sros;
using namespace std;

TEST(PathChecker, CheckIfStartPoseOffset) {
    // 现在车的初始位置为(0, 0, 0),我们按照个这个位置来测试
    auto &s = sros::core::Settings::getInstance();
    s.setTmpValue("main.manual_path_start_pose_check_threshold", "100");
    NavigationPath_vector paths;
    paths.push_back(makeLine(0.09, 0, 1, 0));
    ASSERT_NO_THROW(PathChecker::checkIfStartPoseOffset(paths));
    paths.pop_back();
    paths.push_back(makeLine(0.11, 0, 1, 0));
    ASSERT_ANY_THROW(PathChecker::checkIfStartPoseOffset(paths));
}

TEST(PathChecker, checkIfStartAngleOffset) {
    // 现在车的初始位置为(0, 0, 0),我们按照个这个位置来测试, tan(1) == 0.017455065
    auto &s = sros::core::Settings::getInstance();
    s.setTmpValue("main.manual_path_start_angle_check_threshold", "10");
    NavigationPath_vector paths;
    paths.push_back(makeLine(0, 0, 1, 0.017455064));
    ASSERT_NO_THROW(PathChecker::checkIfStartAngleOffset(paths));
    paths.pop_back();
    paths.push_back(makeLine(0, 0, 1, 0.017455066));
    ASSERT_ANY_THROW(PathChecker::checkIfStartAngleOffset(paths));
}

TEST(PathChecker, CheckIfNotAngleContinuous) {
    // 默认测试3°，第二条路径结束点y的阈值是tan(3) = 0.052407779
    auto &s = sros::core::Settings::getInstance();
    s.setTmpValue("main.manual_path_angle_continuous_check_threshold", "30");
    NavigationPath_vector paths;
    paths.push_back(makeLine(0, 0, 1, 0));
    paths.push_back(makeLine(1, 0, 2, 0.052));
    ASSERT_NO_THROW(PathChecker::checkIfNotAngleContinuous(paths));
    paths.pop_back();
    paths.push_back(makeLine(1, 0, 2, 0.053));
    ASSERT_ANY_THROW(PathChecker::checkIfNotAngleContinuous(paths));
}

TEST(PathChecker, CheckIfNotPoseContinuous) {
    // 默认容忍两条路之间不连续路径为5cm
    auto &s = sros::core::Settings::getInstance();
    s.setTmpValue("main.manual_path_pose_continuous_check_threshold", "50");
    NavigationPath_vector paths;
    paths.push_back(makeLine(0, 0, 1, 0));
    paths.push_back(makeLine(1.049, 0, 2, 0));
    ASSERT_NO_THROW(PathChecker::checkIfNotPoseContinuous(paths));
    paths.pop_back();
    paths.push_back(makeLine(1.051, 0, 2, 0));
    ASSERT_ANY_THROW(PathChecker::checkIfNotPoseContinuous(paths));
}