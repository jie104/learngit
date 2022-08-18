/*
 * @Author: your name
 * @Date: 2020-11-24 15:59:34
 * @LastEditTime: 2021-06-18 10:54:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Editin
 * @FilePath: /sros/modules/navigation/local_map.h
 */
//
// Created by yj on 20-3-16.
//

#ifndef GLOBAL_PLAN_LOCAL_MAP_H
#define GLOBAL_PLAN_LOCAL_MAP_H

#include "lib/include/jps_grid.h"
#include "core/pose.h"
#include "lib/include/navigation.h"
#include "check_collision.h"
class  LocalMap{
public:
    LocalMap(float length,float resolution):length_(length),resolution_(resolution){
        local_map_ = new grid;
    }

    LocalMap():length_(4.0),resolution_(2){
        local_map_ = new grid;
    }

    ~LocalMap();

    void initLocalMap();

// 根据全局地图 生成以当前点为中心的局部地图。
    void getLocalMap(struct grid* global_map,Navigation& nav,sros::core::Pose cur_pose);

    void getLocalMap1(CheckCollision& obs_map,Navigation& nav,sros::core::Pose cur_pose);

    bool convertLocalMapToGlobalMap(sros::core::Pose& local_map_point,sros::core::Pose& global_map_point);
    bool convertGlobalMapToLocalMap(sros::core::Pose& local_map_point,sros::core::Pose& global_map_point);

    struct grid* local_map_;
    float length_;
    int map_width; // 在栅格地图中的宽度。
    float resolution_;
    sros::core::Pose local_left_up_point; // 局部地图左上角在全局地图坐标下的坐标点。
};

#endif //GLOBAL_PLAN_LOCAL_MAP_H
