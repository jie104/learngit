//
// Created by yj on 20-3-27.
//

#ifndef GLOBAL_PLAN_MAP_GRID_H
#define GLOBAL_PLAN_MAP_GRID_H

#include <vector>
#include <queue>
#include "map_cell.h"
#include "core/pose.h"

class MapGrid {
public:

    MapGrid();

    MapGrid(unsigned int size_x, unsigned int size_y);

    ~MapGrid(){}


    inline MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
    }

    void resetPathDist();


    void commonInit();

    unsigned int getIndex(int x, int y);

    inline bool updatePathCell(MapCell* current_cell, MapCell* check_cell,
                                        struct grid*  costmap);

    inline bool updatePathCell1(MapCell* current_cell, MapCell* check_cell,
                               struct grid*  costmap);

    void computeTargetDistance(std::queue<MapCell*>& dist_queue, struct grid* local_map_);

    void setLocalGoal(sros::core::Pose ,struct grid*  costmap);


    inline double unreachableCellCosts() {
        return map_.size() + 1;
    }

    inline double obstacleCosts() {
        return map_.size();
    }

    unsigned int size_x_;
    unsigned int size_y_;
    std::vector<MapCell> map_;

};


#endif //GLOBAL_PLAN_MAP_GRID_H
