/**
 * @file pose_checker.cpp
 *
 * @author pengjiali
 * @date 19-7-5.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "pose_checker.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/src.h"
#include "core/task/task_manager.h"
#include "core/util/utils.h"

using namespace sros::core;

namespace sros {
void PoseChecker::autoDetectStation() {
    auto &settings = sros::core::Settings::getInstance();
    auto enable_auto_detect_station =
        settings.getValue<std::string>("main.enable_auto_detect_station", "True") == "True";

    auto location_ok = (g_state.location_state == sros::core::LOCATION_STATE_RUNNING);

    // 如果启用了自动检测站点，那么当系统处于正常定位状态下执行站点检测逻辑
    if (enable_auto_detect_station && location_ok && !TaskManager::getInstance()->isMovementTaskRunning()) {
        const auto cur_station_no = g_state.station_no;

        // 如果当前站点编号非0，那么当坐标偏离站点一定阈值后就将当前站点清零
        if (cur_station_no != 0) {
            if (!isVehicleOnStation(cur_station_no)) {
                LOGGER(INFO, CMD_HANDER) << "Current station changed:" << g_state.station_no << "-> 0";
                g_state.station_no = 0;
                return;
            }
        }

        // 在系统空闲且当前站点为0，根据当前坐标自动匹配当前站点
        if (cur_station_no == 0 && g_state.isSystemIDLE()) {
            updateVehicleStation();
        }
    }
}

void PoseChecker::detectStationWhenMovementTaskFinished() {
    // 1.判断agv是否到达task的目标站点，若是就设置task的目标站点，结束
    auto task = TaskManager::getInstance()->getMovementTask();
    auto dest_station_id = task->getCurDstStation();
    if (dest_station_id != 0) {
        if (isVehicleOnStation(dest_station_id)) {
            LOGGER(INFO, CMD_HANDER) << "Current station changed: " << g_state.station_no << " -> " << dest_station_id;
            g_state.station_no = dest_station_id;
            return;
        }
    }

    // 2.若agv离task目标站点很远，就检测当前agv到了哪个站点
    updateVehicleStation();
}

bool PoseChecker::checkMovementTaskArriveDest() {
    auto task = TaskManager::getInstance()->getMovementTask();
    auto dst_station_id = task->getCurDstStation();
    auto cur_pose = src_sdk->getCurPose();
    auto &settings = sros::core::Settings::getInstance();
    auto enable_movement_task_arrive_check =
        settings.getValue<std::string>("main.enable_movement_task_arrive_check", "False") == "True";
    if (!enable_movement_task_arrive_check) {
        return true;  // 未开启是否到达目标站点检测，直接返回成功
    }

    auto point_threshold = settings.getValue<double>("main.movement_task_arrive_point_check_threshold", 100) / 1000;
    auto angle_threshold = settings.getValue<double>("main.movement_task_arrive_angle_check_threshold", 50) / 10.0 *
                           DEGREE_TO_RAD;  // 弧度

    auto checkArriveDstPointCheckFunc = [&]() {
        const auto dst_pose = task->getCurDstPose();
        LOGGER(INFO, CMD_HANDER) << "Dst pose is " << dst_pose;

        auto distance = get2PointDistance(dst_pose.x(), dst_pose.y(), cur_pose.x(), cur_pose.y());
        if (distance > point_threshold) {
            LOGGER(WARNING, CMD_HANDER) << "agv not arrive at pose " << dst_pose << ", distance offset: " << distance
                                        << ", threshold: " << point_threshold;
            return false;
        }

        auto offset = std::abs(normalizeYaw(cur_pose.yaw() - dst_pose.yaw()));
        if (offset > angle_threshold) {
            LOGGER(WARNING, CMD_HANDER) << "agv not arrive at pose " << dst_pose << ", angle offset: " << offset
                                        << ", threshold: " << angle_threshold;
            return false;
        }

        LOGGER(INFO, CMD_HANDER) << "agv arrive at pose " << dst_pose << ", offset: " << distance
                                 << ", threshold: " << point_threshold;

        return true;
    };

    LOGGER(INFO, CMD_HANDER) << "Current pose is " << cur_pose;
    if (dst_station_id != 0) {
        auto station = MapManager::getInstance()->getStation(dst_station_id);
        if (station) {
            LOGGER(INFO, CMD_HANDER) << "Station " << dst_station_id << "'s pose is " << station.pos;

            auto distance = get2PointDistance(station.pos.x / 100, station.pos.y / 100, cur_pose.x(), cur_pose.y());
            if (distance > point_threshold) {
                LOGGER(WARNING, CMD_HANDER) << "agv not arrive at station " << dst_station_id
                                            << ", distance offset: " << distance << ", threshold: " << point_threshold;
                return false;
            } else {
                auto offset = std::abs(normalizeYaw(cur_pose.yaw() - station.pos.yaw));
                if (offset > angle_threshold) {
                    LOGGER(WARNING, CMD_HANDER) << "agv not arrive at station " << dst_station_id
                                                << ", angle offset: " << offset << ", threshold: " << angle_threshold;
                    return false;
                }

                LOGGER(INFO, CMD_HANDER) << "agv arrive at station " << dst_station_id << ", offset: " << distance
                                         << ", threshold: " << point_threshold;

                return true;
            }
        } else {
            LOG(INFO) << "Move to a invalid station: " << dst_station_id << ", start dst point check.";
            return checkArriveDstPointCheckFunc();
        }
    } else {  // 检测是否到达了某个位置
        return checkArriveDstPointCheckFunc();
    }

    return true;
}

bool PoseChecker::isVehicleOnStation(const sros::map::StationMark &station) {
    if (!station) {
        return false;
    }

    auto &settings = sros::core::Settings::getInstance();
    auto auto_detect_station_offset = settings.getValue<double>("main.auto_detect_station_offset", 50) / 1000;
    auto cur_pose = src_sdk->getCurPose();
    auto distance = get2PointDistance(station.pos.x / 100, station.pos.y / 100, cur_pose.x(), cur_pose.y());
    if (distance > auto_detect_station_offset) {
        return false;
    }

    auto auto_detect_station_angle_offset =
        settings.getValue<double>("main.auto_detect_station_angle_offset", 300) / 10.0 * DEGREE_TO_RAD;  // 弧度

    auto offset = std::abs(normalizeYaw(cur_pose.yaw() - station.pos.yaw));
    if (offset > auto_detect_station_angle_offset) {
        return false;
    }

    return true;
}

bool PoseChecker::isVehicleOnStation(uint16_t station_id) {
    auto dest_station = MapManager::getInstance()->getStation(station_id);
    return isVehicleOnStation(dest_station);
}

void PoseChecker::updateVehicleStation() {
    auto station_list = MapManager::getInstance()->getStationList();

    for (const auto &s : station_list) {
        if (isVehicleOnStation(s)) {
            if (g_state.station_no != s.id) {
                LOGGER(INFO, CMD_HANDER) << "Current station changed: " << g_state.station_no << " -> " << s.id;
                g_state.station_no = s.id;
            }
            return;
        }
    }

    if (g_state.station_no != 0) {
        LOGGER(INFO, CMD_HANDER) << "Current station changed: " << g_state.station_no << " -> 0";
        g_state.station_no = 0;
    }
}
}  // namespace sros
