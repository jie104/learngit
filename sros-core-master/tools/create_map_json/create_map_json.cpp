
#include <iostream>

// #include "map/NavigationMap.hpp"
#include "../../core/map/NavigationMap.hpp"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./sros_create_map_json map_file.map map_json.json" << std::endl;
        exit(-1);
    }

    std::string map_json_file = argv[2];
    std::string map_file = argv[1];

    std::cout << "mapJson:" << map_json_file << std::endl;
    std::cout << "mapFile:" << map_file << std::endl;

    sros::map::NavigationMap nav_map;
    if (!nav_map.loadFile(map_file))
    {
        std::cout << "Load map file failed: " << map_file << std::endl;
        return -1;
    }

    if (boost::filesystem::exists(map_json_file)) {
        if (!nav_map.updateMapJsonMetaFields(map_json_file)) {
            std::cout << "Update map json file failed: " << map_json_file << std::endl;
            return -1;
        }
    } else {
        if (!nav_map.exportJsonFile(map_json_file.c_str()))
        {
            std::cout << "Save map json file failed: " << map_json_file << std::endl;
            return -1;
        }
    }

    return 0;
}
