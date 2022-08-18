#ifndef ALGORITHM_H
#define ALGORITHM_H

//#include <ompl/base/spaces/ReedsSheppStateSpace.h>
//#include <ompl/base/spaces/DubinsStateSpace.h>
//#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/base/State.h>
//
//typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "collisiondetection.h"
#include "visualization.h"
//#include "reeds_shepp.h"
#include "dubins.h"
#include "reeds_shepp.h"
#include "map_grid.h"
namespace HybridAStar {

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm() {}

 Node3D*  hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                      MapGrid& gridmap);

 Node3D*  findOneRotatePointForAgv(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                      MapGrid& gridmap);

};
}
#endif // ALGORITHM_H
