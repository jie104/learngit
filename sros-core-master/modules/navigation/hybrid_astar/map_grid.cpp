//
// Created by yj on 20-3-27.
//

#include <modules/navigation/lib/include/jps_grid.h>
#include "map_grid.h"

MapGrid::MapGrid()
        : size_x_(0), size_y_(0)
{
}

MapGrid::MapGrid(unsigned int size_x, unsigned int size_y)
        : size_x_(size_x), size_y_(size_y)
{
    commonInit();
}


void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    //ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
            unsigned int id = size_x_ * i + j;
            map_[id].cx = j;
            map_[id].cy = i;
        }
    }
}

void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
        map_[i].target_dist = unreachableCellCosts();
        map_[i].target_mark = false;
        map_[i].within_robot = false;
    }
}

unsigned int MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
}

inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
                                    struct grid*  costmap){

    //if the cell is an obstacle set the max path distance
  //  unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    unsigned  char cost  =  costmap->nodes[check_cell->cy][check_cell->cx].grid_layered_walkable_;
    if(cost == 0x24){
        check_cell->target_dist = obstacleCosts();
       // LOG(INFO)<<"111111111111111该点为障碍物："<<check_cell->cx<<";"<<check_cell->cy;
        return false;
    }

    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) {
        check_cell->target_dist = new_target_dist;
    }
    return true;
}

inline bool MapGrid::updatePathCell1(MapCell* current_cell, MapCell* check_cell,
                                    struct grid*  costmap){

    //if the cell is an obstacle set the max path distance
    //  unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    unsigned  char cost  =  costmap->nodes[check_cell->cy][check_cell->cx].grid_layered_walkable_;
    if(cost == 0x24){
        check_cell->target_dist = obstacleCosts();
        // LOG(INFO)<<"111111111111111该点为障碍物："<<check_cell->cx<<";"<<check_cell->cy;
        return false;
    }

    double new_target_dist = current_cell->target_dist + 1.4;
    if (new_target_dist < check_cell->target_dist) {
        check_cell->target_dist = new_target_dist;
    }
    return true;
}

void MapGrid::computeTargetDistance(std::queue<MapCell*>& dist_queue,  struct grid*  costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    int count=0;
    while(!dist_queue.empty()){
        current_cell = dist_queue.front();
        count++;

        dist_queue.pop();

        if(current_cell->cx > 0){
            check_cell = current_cell - 1;
            if(!check_cell->target_mark){
                //mark the cell as visisted
                check_cell->target_mark = true;
                if(updatePathCell(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }

        if(current_cell->cx < last_col){
            check_cell = current_cell + 1;
            if(!check_cell->target_mark){
                check_cell->target_mark = true;
                if(updatePathCell(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }

        if(current_cell->cy > 0){
            check_cell = current_cell - size_x_;
            if(!check_cell->target_mark){
                check_cell->target_mark = true;
                if(updatePathCell(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }

        if(current_cell->cy < last_row){
            check_cell = current_cell + size_x_;
            if(!check_cell->target_mark){
                check_cell->target_mark = true;
                if(updatePathCell(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }
        if(current_cell->cx > 0&&current_cell->cy > 0){
            check_cell = current_cell - 1-size_x_;
            if(!check_cell->target_mark){
                //mark the cell as visisted
                check_cell->target_mark = true;
                if(updatePathCell1(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }
        if(current_cell->cx > 0&&current_cell->cy < last_row){
            check_cell = current_cell - 1+size_x_;
            if(!check_cell->target_mark){
                //mark the cell as visisted
                check_cell->target_mark = true;
                if(updatePathCell1(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }

        if(current_cell->cx < last_col&&current_cell->cy > 0){
            check_cell = current_cell + 1-size_x_;
            if(!check_cell->target_mark){
                check_cell->target_mark = true;
                if(updatePathCell1(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }
        if(current_cell->cx < last_col&&current_cell->cy < last_row){
            check_cell = current_cell + 1+size_x_;
            if(!check_cell->target_mark){
                check_cell->target_mark = true;
                if(updatePathCell1(current_cell, check_cell, costmap)) {
                    dist_queue.push(check_cell);
                }
            }
        }
    }
    LOG(INFO)<<"计算局部地图距离信息的次数:"<<count;
}


void MapGrid::setLocalGoal(sros::core::Pose pose,struct grid*  costmap){
    std::queue<MapCell*> path_dist_queue;
    MapCell& current = getCell(pose.x(),pose.y());
    current.target_dist = 0.0;
    current.target_mark = true;
    path_dist_queue.push(&current);
    computeTargetDistance(path_dist_queue, costmap);
}


