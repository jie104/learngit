/*
 * @Author: your name
 * @Date: 2021-01-08 10:56:03
 * @LastEditTime: 2021-06-18 10:09:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sros/modules/navigation/check_collision.h
 */
//
// Created by yj on 20-4-17.
//

#ifndef SROS_CHECK_COLLISION_H
#define SROS_CHECK_COLLISION_H
// 主要用来判断导航过程中的障碍物。
// 判断障碍物首先需要有地图。 然后需要有实时障碍物。
// 为了满足多线程使用需要。一般情况下，当进来实时障碍物时，更新地图需要锁住地图资源。
//
#include <iostream>
#include <mutex>

#include "core/pose.h"
#include "lib/include/navigation.h"

typedef struct{
    uint8_t value;
    uint8_t type;
}MAP_POINT;

typedef struct{
    sros::core::Pose obs_pose;
    std::string obs_name;
}COLLISION_OBS;


class CheckCollision {
public:



    CheckCollision() {collision_map = nullptr;}

    // 析构函数，需要释放地图资源。
    ~CheckCollision();

    // 需要初始化地图
    void init_check_collision(sros::map::NavigationMap_ptr nav_map,NavConfig* cfg);

    // 根据每次进来的实时障碍点来更新地图。更新地图前需要删除掉之前的临时障碍点。

    void update_collision_map(Navigation &nav, std::map<std::string,std::vector<Eigen::Vector2d>> &cur_obstalces);


    bool find_obstacle_type(uint8_t id,std::string& device_name);

    uint8_t find_obstacle_no(std::string& device_name);


    // 判断矩形区域内是否有障碍物。有 返回true 没有返回 false。
    bool check_point_collision(Navigation &nav, sros::core::Pose cur_map_pose, float robot_length, float robot_width,COLLISION_OBS& obs_pose);


    // 建立一个互斥锁。
    std::mutex mtx;
    uint8_t init_or_not = 0;
    // 上次的临时障碍点。
    std::vector<Eigen::Vector2d> last_tmp_obstalces;
    MAP_POINT **collision_map;
    NavConfig *cfg_;

    std::map<std::string,uint8_t> str_to_int_map_;
    std::map<uint8_t,std::string> int_to_str_map_;
    uint8_t curr_incre_ = 1;

    float map_height;
    float map_width;

    
};

typedef std::shared_ptr<CheckCollision>  CheckCollision_Ptr;

#endif //SROS_CHECK_COLLISION_H
