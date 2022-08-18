/**
 * @file pose_checker.h
 *
 * @author pengjiali
 * @date 19-7-5.
 *
 * @describe 一些位置检测相关的逻辑
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MODULES_MAIN_POSE_CHECKER_H_
#define MODULES_MAIN_POSE_CHECKER_H_


#include "core/map_manager.h"

namespace sros {

class PoseChecker {
 public:
    static void autoDetectStation();

    /**
     * @brief 当移动任务结束后检测agv到达的站点
     * 1.判断agv是否到达task的目标站点，若是就设置task的目标站点，结束
     * 2.若agv离task目标站点很远，就检测当前agv到了哪个站点
     */
    static void detectStationWhenMovementTaskFinished();

    /**
     * @brief 检测移动任务是否到达目的地
     * @return
     */
    static bool checkMovementTaskArriveDest();

 private:
    static bool isVehicleOnStation(const sros::map::StationMark &station);

    static bool isVehicleOnStation(uint16_t station_id);

    static void updateVehicleStation();
};

}  // namespace sros

#endif  // MODULES_MAIN_POSE_CHECKER_H_
