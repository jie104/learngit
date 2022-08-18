//
// Created by lfc on 2022/3/5.
//
#include "NavigationMap.hpp"


int main(int argc, char* argv[]) {
    sros::map::NavigationMap nav_map;
    nav_map.loadFile("/sros/华天慧创三楼opt.map",true);
    LOG(INFO) << "begin save";
    auto nav_m = nav_map.getMapData();
    float new_resolution = 4;
    int offset_x,offset_y;
    offset_x = nav_map.getMapZeroOffsetX();
    offset_y = nav_map.getMapZeroOffsetY();
    double world_x,world_y;
    int new_offset_x,new_offset_y;
    world_x = (0 - offset_x) * 2 / 100.0;
    world_y = (offset_y - 0) * 2 / 100.0;
    new_offset_x = floorf(0 - world_x * 100.0 / new_resolution + 0.5f);
    new_offset_y = floorf(world_y * 100.0 / new_resolution + 0.5f);
    auto width = nav_map.getMapSize().x;
    auto height = nav_map.getMapSize().y;
    int new_width,new_height;
    world_x = (width-1 - offset_x) * 2 / 100.0;
    world_y = (offset_y - height+1) * 2 / 100.0;
    new_width = floorf(new_offset_x + world_x * 100.0 / new_resolution + 0.5f) + 1;
    new_height = floorf(new_offset_y - world_y * 100.0 / new_resolution + 0.5f) + 1;
    sros::map::PyramidNavMap_ptr inflation_map_data,tmp_data;
    inflation_map_data.reset(
        new sros::map::PyramidNavMap<sros::map::NavMapNode_t>(new_width, new_height, offset_x, offset_y, new_resolution/100.0f));
    tmp_data.reset(
        new sros::map::PyramidNavMap<sros::map::NavMapNode_t>(new_width, new_height, offset_x, offset_y, new_resolution/100.0f));
    LOG(INFO) << "begin to map";
    nav_m->cpToCoarseMap(inflation_map_data);
    LOG(INFO) << "end to map";
    int index = 0;
    for (int i = 0; i < new_width; ++i) {
        for (int j = 0; j < new_height; ++j) {
            if (!inflation_map_data->isGridFree(i, j)) {
                index++;
            }
        }
    }
    LOG(INFO) << "index:" << index << "," << new_width * new_height;
    unsigned char *tt;
    int map_data_length = new_width * new_height;
    tt = new unsigned char[map_data_length];
    inflation_map_data->cpToArray(tt, (unsigned char)1, (unsigned char)0);

    nav_map.exportMapPngFiles("test");
}