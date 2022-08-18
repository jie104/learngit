/**
 * @file real_time_detect_info.cpp
 *
 * @author pengjiali
 * @date 19-7-9.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "real_time_detect_info.h"
#include <glog/logging.h>
#include "core/util/time.h"
#include "core/device/device.h"

const int fresh_time_ms = 200;  // 多少时间内算是有效的
const std::string LASER_NAME_HEAD = sros::device::DEVICE_LIDAR;


namespace network {

RealTimeDetectInfo::RealTimeDetectInfo() {
}

RealTimeDetectInfo::~RealTimeDetectInfo() {

}

void RealTimeDetectInfo::setObstacleInfo(sros::core::ObstacleMsg_ptr ptr) {

    //LOG(INFO) << "ptr->oba_name: " << ptr->oba_name;
    if(ptr->oba_name.empty()) {
        LOG(INFO) << "oba_name is empty, return";
        return;
    }

    if(ptr->oba_name.compare(0, LASER_NAME_HEAD.size(), LASER_NAME_HEAD) == 0) {
        laser_name_ = ptr->oba_name;
    }

    if(map_laser_obstacle_info_.find(ptr->oba_name) == map_laser_obstacle_info_.end()) {
        map_laser_obstacle_info_[ptr->oba_name] = std::make_shared<StObstacleInfo>();
        LOG(INFO) << "make_shared1: " << ptr->oba_name;
    }

    map_laser_obstacle_info_[ptr->oba_name]->obstacle_update_time_ = sros::core::util::get_time_in_ms();
    map_laser_obstacle_info_[ptr->oba_name]->obstacle_ptr_ = ptr;

}

void RealTimeDetectInfo::setAvoidObstacleInfo(sros::core::ObstacleMsg_ptr ptr) {

    //LOG(INFO) << "ptr->avoid_oba_name: " << ptr->oba_name;
    if(ptr->oba_name.empty()) {
        LOG(INFO) << "avoid_oba_name is empty, return";
        return;
    }

    if(map_laser_obstacle_info_.find(ptr->oba_name) == map_laser_obstacle_info_.end()) {
        map_laser_obstacle_info_[ptr->oba_name] = std::make_shared<StObstacleInfo>();
        LOG(INFO) << "make_shared2: " << ptr->oba_name;
    }

    map_laser_obstacle_info_[ptr->oba_name]->avoid_obstacle_update_time_ = sros::core::util::get_time_in_ms();
    map_laser_obstacle_info_[ptr->oba_name]->avoid_obstacle_ptr_ = ptr;
}

void RealTimeDetectInfo::setLaserScanInfo(sros::core::LaserScan_ptr ptr) {
        laser_scan_update_time_ = sros::core::util::get_time_in_ms();
        laser_scan_ptr_ = ptr;
}

void RealTimeDetectInfo::setLmkMatchInfo(sros::core::LmkMatchinfoMsg_ptr ptr) {
    lmk_match_info_update_time_ = sros::core::util::get_time_in_ms();
    lmk_match_info_ptr_ = ptr;
}

sros::core::ObstacleMsg_ptr RealTimeDetectInfo::getObstacleInfo(std::string laser_name) {
    if(map_laser_obstacle_info_.find(laser_name) == map_laser_obstacle_info_.end()) {
        return nullptr;
    }

    auto obstacle_update_time = map_laser_obstacle_info_[laser_name]->obstacle_update_time_;
    auto elapse_ms = sros::core::util::get_time_in_ms() - obstacle_update_time;
    if (elapse_ms > 0 && elapse_ms <= fresh_time_ms) {
        return map_laser_obstacle_info_[laser_name]->obstacle_ptr_;
    }

    return nullptr;
}

std::map<std::string, sros::core::ObstacleMsg_ptr> RealTimeDetectInfo::getExtObstacleInfo() {
    map_ext_obstacle_info_.clear();

    for (auto &iter : map_laser_obstacle_info_) {
        if(iter.first != laser_name_ && iter.first != sros::device::DEVICE_R2100) {
            auto elapse_ms = sros::core::util::get_time_in_ms() - iter.second->obstacle_update_time_;
            if (elapse_ms > 0 && elapse_ms <= fresh_time_ms) {
                map_ext_obstacle_info_[iter.first] = iter.second->obstacle_ptr_;
            }
        }
    }
    return map_ext_obstacle_info_;
}


std::map<std::string, sros::core::ObstacleMsg_ptr> RealTimeDetectInfo::getAvoidObstacleInfo(){
    map_avoid_obstacle_info_.clear();

    for (auto &iter : map_laser_obstacle_info_) {
        auto elapse_ms = sros::core::util::get_time_in_ms() - iter.second->avoid_obstacle_update_time_;
        if (elapse_ms > 0 && elapse_ms <= fresh_time_ms) {
            map_avoid_obstacle_info_[iter.first] = iter.second->avoid_obstacle_ptr_;
        }
    }

    return map_avoid_obstacle_info_;
}

sros::core::LaserScan_ptr RealTimeDetectInfo::getLaserScanInfo() const {
    auto elapse_ms = sros::core::util::get_time_in_ms() - laser_scan_update_time_;
    if (elapse_ms > 0 && elapse_ms <= fresh_time_ms) {
        return laser_scan_ptr_;
    }

    return nullptr;
}

sros::core::LmkMatchinfoMsg_ptr RealTimeDetectInfo::getLmkMatchInfo() const {
    return lmk_match_info_ptr_;  // lmk match数据是一直保持的，若没有变就不会更新
}

}