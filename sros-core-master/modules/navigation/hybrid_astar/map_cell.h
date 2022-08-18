//
// Created by yj on 20-3-27.
//

#ifndef GLOBAL_PLAN_MAP_CELL_H
#define GLOBAL_PLAN_MAP_CELL_H


class MapCell {
public:
    /**
     * @brief  Default constructor
     */
    MapCell();

    /**
     * @brief  Copy constructor
     * @param mc The MapCell to be copied
     */
    MapCell(const MapCell& mc);

    unsigned int cx, cy; ///< @brief Cell index in the grid map

    double target_dist; ///< @brief Distance to planner's path

    bool target_mark; ///< @brief Marks for computing path/goal distances

    bool within_robot; ///< @brief Mark for cells within the robot footprint
};


#endif //GLOBAL_PLAN_MAP_CELL_H
