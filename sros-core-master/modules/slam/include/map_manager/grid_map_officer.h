//
// Created by lfc on 17-7-11.
//

#ifndef SROS_GRID_MAP_OFFICER_H
#define SROS_GRID_MAP_OFFICER_H
#include <vector>
#include <memory>
#include "point_container.h"
namespace mapping{
class GridMapManager;
class GridMapOfficer {
public:
    GridMapOfficer(float min_resolution_, int level, float update_factor_free = 0.4, bool update_map = true);


    bool loadMap(std::string map_name);

    bool saveMap(std::string map_name);

    void updateMap(PointContainer_Ptr points, bool is_update_free = true);
private:
    GridMapOfficer(){

    }

    std::vector<std::shared_ptr<GridMapManager>> maps_manager;
    int map_level;

};

typedef std::shared_ptr<GridMapOfficer> GridMapOfficer_Ptr;
}



#endif //SROS_GRID_MAP_OFFICER_H
