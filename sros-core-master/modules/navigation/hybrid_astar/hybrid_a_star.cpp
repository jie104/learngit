//
// Created by yj on 19-9-28.
//
// 该部分主要通过hybrid a star 算法计算全局路径。输入为 七点和终点坐标，以及栅格地图，输出为全局路径点。
#include "hybrid_a_star.h"
#include "iostream"
#include <chrono>
#include <modules/navigation/nav_config.h>
#include <modules/navigation/check_collision.h>

using  namespace HybridAStar;
//###################################################
//                                      PLAN THE PATH
//###################################################

hybrid_a_star::hybrid_a_star()
{

}

void hybrid_a_star::Init(int width,int height){
    // 需要初始化
    //
    int depth = Constants::headings;
    length = width * height * depth;
    nodes2D = new Node2D[1]();

    int64_t now2 = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000L;
    try{
        nodes3D= new Node3D[length]();
    }catch ( const std::bad_alloc& e ) {
        LOG(INFO)<<"-----------------申请内存失败！";
        return ;
    }
    int64_t end2 = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000L;

    LOG(INFO)<<"new构建nodes3d的时间: ms:"<<end2-now2;
}


void hybrid_a_star::InitConfig(NavConfig& cfg){
    // 需要初始化
    //
    configurationSpace.InitConfig(cfg);

}


void hybrid_a_star::tracePath(const Node3D* node, int i) {
    if (node == nullptr) {
        return;
    }

    i++;
   // LOG(INFO)<<"hybrid a star traj: "<<node->getX()<<";"<<node->getY();
    path.push_back(*node);
    tracePath(node->getPred(), i);
}

void hybrid_a_star::plan(sros::core::Pose start,sros::core::Pose goal,struct grid& grid_map) {
    // if a start as well as goal are defined go ahead and plan

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid_map.width;
    int height = grid_map.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    LOG(INFO)<<"局部地图长宽深度尺寸:"<<width<<":"<<height<<":"<<depth;
    // 复位每个节点的值。
    resetNode();
    // ________________________
    // retrieving goal position
    float x = goal.x();
    float y = goal.y();
    float t = goal.yaw();

    //    x = -9.48;
    //   y = 5.15;
    LOG(INFO)<<"hybrid 终点坐标："<<x<<";"<<y<<";"<<t;
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);
    // _________________________
    // retrieving start position
    x = start.x();
    y = start.y();
    t = start.yaw();
    //    x =4.83;
    //    y =-4.18;
    LOG(INFO)<<"hybrid 起点坐标："<<x<<";"<<y<<";"<<t;
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr,4);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);
    float dubinsLookup = 0;
    configurationSpace.updateGrid(&grid_map);

    LOG(INFO)<<"开始执行hybrid a star  ";

    // 计算局部地图到达目标点的dist。
    MapGrid map_grid((unsigned int)grid_map.width,(unsigned int)grid_map.height);
    map_grid.resetPathDist();
    sros::core::Pose local_target_pose(nGoal.getX(),nGoal.getY());

    int64_t now11 = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000L;
    map_grid.setLocalGoal(local_target_pose ,&grid_map);
    int64_t end11 = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000L;

    LOG(INFO)<<"000000000000000000计算距离地图的时间："<<end11-now11<<"ms";

    // FIND THE PATH
    Node3D* nSolution = algorithm_.hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,configurationSpace, &dubinsLookup,map_grid);

    path.clear();
    // TRACE THE PATH
    // smoother.tracePath(nSolution);
   // LOG(INFO)<<"1123456  ";
    tracePath(nSolution);

}

void hybrid_a_star::findOneRotatePointForAgv(sros::core::Pose start,sros::core::Pose goal,struct grid& grid_map) {
    // if a start as well as goal are defined go ahead and plan
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid_map.width;
    int height = grid_map.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    LOG(INFO)<<"局部地图长宽深度尺寸:"<<width<<":"<<height<<":"<<depth;
    // 复位每个节点的值。
    resetNode();
    // retrieving goal position
    float x = goal.x();
    float y = goal.y();
    float t = goal.yaw();

    LOG(INFO)<<"hybrid 终点坐标："<<x<<";"<<y<<";"<<t;
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);

    x = start.x();
    y = start.y();
    t = start.yaw();
    LOG(INFO)<<"hybrid 起点坐标："<<x<<";"<<y<<";"<<t;
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr,4);
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);
    float dubinsLookup = 0;
    configurationSpace.updateGrid(&grid_map);

    LOG(INFO)<<"开始执行hybrid a star  ";

    // 计算局部地图到达目标点的dist。
    MapGrid map_grid((unsigned int)grid_map.width,(unsigned int)grid_map.height);
    map_grid.resetPathDist();
    sros::core::Pose local_target_pose(nGoal.getX(),nGoal.getY());

    int64_t now11 = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000L;
    map_grid.setLocalGoal(local_target_pose ,&grid_map);
    int64_t end11 = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000L;

    LOG(INFO)<<"000000000000000000计算距离地图的时间："<<end11-now11<<"ms";

    // FIND THE PATH
    Node3D* nSolution = algorithm_.findOneRotatePointForAgv(nStart, nGoal, nodes3D, nodes2D, width, height,configurationSpace, &dubinsLookup,map_grid);

    path.clear();
    tracePath(nSolution);
}


