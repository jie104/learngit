/**
 * @file real_time_detect_info.h
 *
 * @author pengjiali
 * @date 19-7-9.
 *
 * @describe
 * 存放小车实时检测到的信息，比如：雷达点、信标。放到一起供上层一次性获取
 *
 * @note 不是线程安全的，现在是一个线程访问，没有加锁
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MODULES_NETWORK_REAL_TIME_DETECT_INFO_H
#define SROS_MODULES_NETWORK_REAL_TIME_DETECT_INFO_H

#include "core/msg/ObstacleMsg.hpp"
#include "core/msg/laser_scan_msg.hpp"
#include "core/msg/lmk_match_info_msg.hpp"

namespace network {

enum LaserNo {
    Laser_0, // 第一个雷达
    Laser_1,
    Laser_tof,
    Laser_eu100,
    Camera_d435
};

class RealTimeDetectInfo {
 public:
    RealTimeDetectInfo();
    ~RealTimeDetectInfo();
    void setObstacleInfo(sros::core::ObstacleMsg_ptr ptr);
    void setAvoidObstacleInfo(sros::core::ObstacleMsg_ptr ptr);
    void setLaserScanInfo(sros::core::LaserScan_ptr ptr);
    void setLmkMatchInfo(sros::core::LmkMatchinfoMsg_ptr ptr);

    std::string getLaserName() { return laser_name_; }

    sros::core::ObstacleMsg_ptr getObstacleInfo(std::string laser_name);
    std::map<std::string, sros::core::ObstacleMsg_ptr> getExtObstacleInfo();
    std::map<std::string, sros::core::ObstacleMsg_ptr> getAvoidObstacleInfo();

    sros::core::LaserScan_ptr getLaserScanInfo() const;
    sros::core::LmkMatchinfoMsg_ptr getLmkMatchInfo() const;

 private:

    struct StObstacleInfo {
        uint64_t obstacle_update_time_;
        sros::core::ObstacleMsg_ptr obstacle_ptr_;

        uint64_t avoid_obstacle_update_time_;
        sros::core::ObstacleMsg_ptr avoid_obstacle_ptr_;

        StObstacleInfo() {
            obstacle_update_time_ = 0;
            obstacle_ptr_ = nullptr;

            avoid_obstacle_update_time_ = 0;
            avoid_obstacle_ptr_ = nullptr;
        }
    };
    typedef std::shared_ptr<StObstacleInfo> StObstacleInfo_Ptr;

    // unit ms
    // 雷达0
    std::string laser_name_;                //前车雷达的名称
    uint64_t laser_scan_update_time_ = 0;
    sros::core::LaserScan_ptr laser_scan_ptr_;

    // 雷达1 R2100
    uint64_t laser_scan1_update_time_ = 0;
    sros::core::LaserScan_ptr laser_scan1_ptr_;

    std::map<std::string, StObstacleInfo_Ptr> map_laser_obstacle_info_;

    uint64_t lmk_match_info_update_time_ = 0;
    sros::core::LmkMatchinfoMsg_ptr lmk_match_info_ptr_;

    std::map<std::string, sros::core::ObstacleMsg_ptr> map_ext_obstacle_info_;
    std::map<std::string, sros::core::ObstacleMsg_ptr> map_avoid_obstacle_info_;
};
}  // namespace network

#endif  // SROS_MODULES_NETWORK_REAL_TIME_DETECT_INFO_H
