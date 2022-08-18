//
// Created by yj on 19-6-14.
//

#ifndef SROS_COSTMAP_MODEL_H
#define SROS_COSTMAP_MODEL_H

#include <modules/navigation/lib/include/jps_grid.h>
#include "core/pose.h"
#include "line_iterator.h"

class CostmapModel {
public:
    /**
     * @brief  Constructor for the CostmapModel
     * @param costmap The costmap that should be used
     * @return
     */
    CostmapModel( struct grid& ma);

    /**
     * @brief  Destructor for the world model
     */
     ~CostmapModel(){}

    /**
     * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
     * @param  position The position of the robot in world coordinates
     * @param  footprint The specification of the footprint of the robot in world coordinates
     * @param  inscribed_radius The radius of the inscribed circle of the robot
     * @param  circumscribed_radius The radius of the circumscribed circle of the robot
     * @return Positive if all the points lie outside the footprint, negative otherwise
     */
     double footprintCost(const sros::core::Pose& position,sros::core::grid_point offset, const std::vector<sros::core::Pose>& footprint,
                                 double inscribed_radius, double circumscribed_radius);

    /**
     * @brief  Rasterizes a line in the costmap grid and checks for collisions
     * @param x0 The x position of the first cell in grid coordinates
     * @param y0 The y position of the first cell in grid coordinates
     * @param x1 The x position of the second cell in grid coordinates
     * @param y1 The y position of the second cell in grid coordinates
     * @return A positive cost for a legal line... negative otherwise
     */
    double lineCost(int x0, int x1, int y0, int y1) ;

    /**
     * @brief  Checks the cost of a point in the costmap
     * @param x The x position of the point in cell coordinates
     * @param y The y position of the point in cell coordinates
     * @return A positive cost for a legal point... negative otherwise
     */
    double pointCost(int x, int y) ;



private:
     struct grid costmap_; ///< @brief Allows access of costmap obstacle information

};

#endif //SROS_COSTMAP_MODEL_H
