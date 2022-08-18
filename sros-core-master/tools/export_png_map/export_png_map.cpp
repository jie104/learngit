//
// Created by lhx on 18-4-4.
//

#include <iostream>

//#include "map/NavigationMap.hpp"
#include "../../core/map/NavigationMap.hpp"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./sros_export_png_map ori.map export.png" << std::endl;
        exit(-1);
    }

    sros::map::NavigationMap nav_map;

    nav_map.loadFile(argv[1]);
    nav_map.exportMapPngFiles(argv[2]);
    return 0;
}
