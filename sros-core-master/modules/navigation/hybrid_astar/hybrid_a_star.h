//
// Created by yj on 19-9-28.
//

#ifndef GLOBAL_PLAN_HYBRID_A_STAR_H
#define GLOBAL_PLAN_HYBRID_A_STAR_H

#include "constants.h"
#include "helper.h"
#include "node3d.h"
#include "node2d.h"
#include "algorithm.h"
#include "core/pose.h"
#include "visualization.h"
#include "collisiondetection.h"
#include "reeds_shepp.h"
#include "map_grid.h"
#include <modules/navigation/nav_config.h>

namespace   HybridAStar {
struct gridnode
{
    double gScore;
    double fScore;
    int8_t id   ;// 1表示在open set  -1 表示clostset

};

class hybrid_a_star {

public:
    hybrid_a_star();

    void Init(int width,int height);

    void InitConfig(NavConfig& cfg_);

    void deleteNode(){
        delete[] nodes3D;
        delete[] nodes2D;
    }

    void resetNode(){
        for(int i=0;i<length;i++){
            nodes3D[i].resetNode();
        }
    }

    void plan(sros::core::Pose start, sros::core::Pose goal,struct grid& grid_map);

    void findOneRotatePointForAgv(sros::core::Pose start, sros::core::Pose goal,struct grid& grid_map);

    void tracePath(const Node3D* node, int i = 0);
    //struct grid grid_map_;
    CollisionDetection configurationSpace;
    Algorithm algorithm_;
    std::vector<Node3D> path;
    Node3D* nodes3D;
    Node2D* nodes2D;
    int length;
};
}

#endif //GLOBAL_PLAN_HYBRID_A_STAR_H
