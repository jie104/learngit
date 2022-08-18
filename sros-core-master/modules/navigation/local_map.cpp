/*
 * @Author: your name
 * @Date: 2020-11-24 15:59:34
 * @LastEditTime: 2021-07-18 10:08:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sros/modules/navigation/local_map.cpp
 */
//
// Created by yj on 20-3-16.
//

#include "local_map.h"


LocalMap:: ~LocalMap(){
    for(int i=0;i<local_map_->height;i++){
        delete [] local_map_->nodes[i];
    }
    delete [] local_map_->nodes;

    delete [] local_map_;
}

void LocalMap::initLocalMap(){



    local_map_->width = (int)(length_*100.0/resolution_*2);
    local_map_->height = (int)(length_*100.0/resolution_*2);
    map_width = (int)(length_*100.0/resolution_*2);
    LOG(INFO)<<"1111local_map_:height和width:"<<local_map_->width;
    //struct node **nodes;
    //nodes = (struct node **) malloc(local_map->height * sizeof(struct node *));
    local_map_->nodes= new  node*[local_map_->height];
     for (int i=0;i<local_map_->height;i++) {
         local_map_->nodes[i] = new  node[local_map_->width];
     }
     //local_map_->nodes = nodes;
}

// 根据全局地图 生成以当前点为中心的局部地图。
void LocalMap::getLocalMap(struct grid* global_map,Navigation& nav,sros::core::Pose cur_pose){

    int offset_x = nav.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = nav.getNavigationMap()->getMapZeroOffsetY();
    LOG(INFO)<<"offset_x:"<<offset_x<<";offset_y:"<<offset_y;
    int icur_x = 0;
    int icur_y = 0;
    nav.convertMapCoords(offset_x, offset_y, cur_pose.x(), cur_pose.y(), &icur_x, &icur_y);
    LOG(INFO)<<"icur_x:"<<icur_x<<";icur_y:"<<icur_y;
    int globa_map_x =icur_x-local_map_->width/2;
    int globa_map_y =icur_y-local_map_->height/2;

    local_left_up_point.x() = icur_x-local_map_->width/2;
    local_left_up_point.y() = icur_y-local_map_->height/2;

    LOG(INFO)<<"globa_map_x:"<<globa_map_x<<";globa_map_y:"<<globa_map_y;
    for(int i = 0;i<local_map_->height;i++){
        for(int j = 0;j<local_map_->width;j++)
        {
            if( (globa_map_y+i)<0||(globa_map_x+j)<0||(globa_map_y+i)>=global_map->height||(globa_map_x+j)>=global_map->width){
                local_map_->nodes[i][j].grid_layered_walkable_ =     INIT_WALKABLE_FALSE ;
            } else {
                local_map_->nodes[i][j].grid_layered_walkable_ = global_map->nodes[globa_map_y+i][globa_map_x+j].grid_layered_walkable_;
            }
        }
    }
}

// 根据全局地图 生成以当前点为中心的局部地图。
void LocalMap::getLocalMap1(CheckCollision& obs_map,Navigation& nav,sros::core::Pose cur_pose){

    int offset_x = nav.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = nav.getNavigationMap()->getMapZeroOffsetY();
    LOG(INFO)<<"offset_x:"<<offset_x<<";offset_y:"<<offset_y;
    int icur_x = 0;
    int icur_y = 0;
    nav.convertMapCoords(offset_x, offset_y, cur_pose.x(), cur_pose.y(), &icur_x, &icur_y);
    LOG(INFO)<<"icur_x:"<<icur_x<<";icur_y:"<<icur_y;
    int globa_map_x =ceil(icur_x-local_map_->width/2);
    int globa_map_y =ceil(icur_y-local_map_->height/2);

    local_left_up_point.x() = ceil(icur_x-local_map_->width/2);
    local_left_up_point.y() = ceil(icur_y-local_map_->height/2);
    
    int count=0;
    int count1=0;
    int count2=0;

    LOG(INFO)<<"globa_map_x:"<<globa_map_x<<";globa_map_y:"<<globa_map_y;
    for(int i = 0;i<local_map_->height;i++){
        for(int j = 0;j<local_map_->width;j++)
        {
            if( (globa_map_y+i)<0||(globa_map_x+j)<0||(globa_map_y+i)>=obs_map.map_height||(globa_map_x+j)>=obs_map.map_width){
                local_map_->nodes[i][j].grid_layered_walkable_ = INIT_WALKABLE_FALSE ;
                count++;
            } else {
                if(obs_map.collision_map[globa_map_x+j][globa_map_y+i].value>0){
                    local_map_->nodes[i][j].grid_layered_walkable_ = INIT_WALKABLE_FALSE;
                    count1++;
                }else{
                   local_map_->nodes[i][j].grid_layered_walkable_ = INIT_WALKABLE_TRUE; 
                   count2++;
                }
                
            }
        }
    }
    LOG(INFO)<<"局部地图超过边界的点数："<<count<<" ;有障碍的点数："<<count1<<"; 无障碍的点数："<<count2;
}


// 需要将局部地图坐标转换为全局地图坐标。

bool LocalMap::convertLocalMapToGlobalMap(sros::core::Pose& local_map_point,sros::core::Pose& global_map_point)
{

    global_map_point.x() = local_map_point.x()+local_left_up_point.x();
    global_map_point.y() = local_map_point.y()+local_left_up_point.y();
    return true;
}

// 将全局地图坐标转化为局部地图坐标。
bool LocalMap::convertGlobalMapToLocalMap(sros::core::Pose& local_map_point,sros::core::Pose& global_map_point)
{
    local_map_point.x() = global_map_point.x()-local_left_up_point.x();
    local_map_point.y() = global_map_point.y()-local_left_up_point.y();
    if(local_map_point.x()<0||local_map_point.y()<0){
        return false;
    }
    return true;
}


//将全局世界坐标 转为 局部地图坐标。用于规划hybrid astar