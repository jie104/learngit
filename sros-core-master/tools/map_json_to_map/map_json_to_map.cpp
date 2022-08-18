
#include <iostream>
#include <boost/filesystem.hpp>

// #include "map/NavigationMap.hpp"
#include "../../core/map/NavigationMap.hpp"

bool updateMapFile(const std::string map_json_file, const std::string &map_file)
{
    boost::filesystem::path map_file_path(map_file);
    std::string map_name = map_file_path.stem().string();
    std::string map_path = map_file_path.parent_path().string();
    std::string dest_json_file = map_path + "/" + map_name + ".json";
    sros::map::NavigationMap nav_map;
    nav_map.loadFile(map_file);

    // 更新绘图地图数据
    try {
        if (!nav_map.importJsonFile(map_json_file.c_str()))
        {
            return false;
        }
    } catch (std::exception &e) {
        std::cerr << "Failed to load json file: " << map_json_file << " " << e.what() << std::endl;
        return false;
    }

    if (nav_map.clearArea(map_json_file.c_str()))
    {
        std::cout << "save map png file: " << map_name << std::endl;
        nav_map.exportMapPngFiles(map_name.c_str());
    }

    std::string cmd = "mv " + map_json_file + " " + dest_json_file;
    nav_map.saveFile(map_file.c_str());
    system(cmd.c_str());
    return true;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./sros_map_json_to_map map_json.json map_file.map" << std::endl;
        exit(-1);
    }

    std::string map_json_file = argv[1];
    std::string map_file = argv[2];

    std::cout << "mapJson:" << map_json_file << std::endl;
    std::cout << "mapFile:" << map_file << std::endl;

    if (!updateMapFile(map_json_file, map_file))
    {
        std::cerr << "Failed to save map file" << std::endl;
        return -1;
    }

    return 0;
}
