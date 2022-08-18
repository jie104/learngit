//
// Created by lhx on 16-1-6.
//

#include "map_manager.h"

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <boost/thread/lock_guard.hpp>
#include "core/state.h"
#include "core/logger.h"
#include "core/exec_error.hpp"

namespace sros {
namespace core {

using namespace std;

string MapManager::MAP_SAVE_PATH = "/sros/map/";

MapManager::MapManager() { freshMapList(); };

MapManager::~MapManager(){

};

MapManager *MapManager::getInstance() {
    static MapManager singleton;
    return &singleton;
}

void MapManager::freshMapList() {
    using namespace boost::filesystem;
    boost::lock_guard<boost::shared_mutex> look(mutex_map_list_);

    MapInfo map;
    map_infos_.clear();

    path map_extension(".map");
    directory_iterator end;
    // 此处出现过一次异常，但不知道为啥，暂时用try，catch捕获一下，确认是什么错误。
    // 崩溃函数：boost::filesystem::detail::directory_iterator_construct(boost::filesystem::directory_iterator&,
    // boost::filesystem::path const&, boost::system::error_code*) ()
    try {
        for (directory_iterator pos(MAP_SAVE_PATH); pos != end; ++pos) {
            path p = pos->path();

            if (p.has_filename() && p.extension() == map_extension) {
                map.name_ = p.stem().string();
                map.file_path_ = p.string();
                map.raw_pgm_path_ = getGrayMapPath(map.name_);
                map.create_time_ = 0;
                map.last_modified_time_ = last_write_time(p);

                map_infos_[map.name_] = map;
            }
        }
    } catch (const filesystem_error &e) {
        LOG(ERROR) << "文件遍历失败： " << e.what();
    } catch (...) {
        LOG(ERROR) << "未知错误！";
    }
};

std::vector<std::string> MapManager::getMapListSortByName() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_map_list_);
    std::vector<std::string> map_list;
    for (auto item : map_infos_) {
        map_list.push_back(item.second.name_);
    }
    std::sort(map_list.begin(), map_list.end());
    return map_list;
}

MapVector_t MapManager::getMapInfoList() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_map_list_);
    MapVector_t map_info_list;
    for (auto item : map_infos_) {
        map_info_list.push_back(item.second);
    }
    return map_info_list;
}

MapInfo MapManager::getMapInfo(const std::string &name) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_map_list_);

    auto search = map_infos_.find(name);
    if (search != map_infos_.end()) {
        return search->second;
    }

    LOG(ERROR) << "Can not get map name : " << name;
    return MapInfo();
}

void MapManager::mapUpdated(const std::string &map_name) {
    if (g_state.getCurMapName() == map_name) {
        setCurrentMap(map_name);  // 当前地图被更新了，更细当前缓存在内存中的地图
    }
}

bool MapManager::setCurrentMap(const std::string &map_name) {
    boost::lock_guard<boost::shared_mutex> lock(mutex_cur_map_);

    is_load_map_ = false;
    if (map_name == NO_MAP) {
        cur_navigation_map_.reset();
        return false;
    }

    if (!cur_navigation_map_) {
        cur_navigation_map_ = std::make_shared<sros::map::NavigationMap>();
    }

    // .json地图 >= .map地图，若没有.json地图，加载地图列表的时候会生成.json地图
    std::string json_map_file_path = MAP_SAVE_PATH + map_name + ".json";
    try {
        if (!cur_navigation_map_->importJsonFile(json_map_file_path.c_str())) {
            SET_ERROR(ERROR_CODE_LOAD_MAP_FAILED, "Failed to load json file::", json_map_file_path);
            return false;
        }
    } catch (std::exception &e) {
        SET_ERROR(ERROR_CODE_LOAD_MAP_FAILED, "Failed to load json file::", json_map_file_path, e.what());
        return false;
    }

    //   is_clear_area // 擦除区域的处理在sros_map_json_to_map中处理了,此处不需要处理

    LOG(INFO) << "Loaded current map: " << map_name;
    is_load_map_ = true;

    return true;
}

sros::map::StationMark MapManager::getStation(uint16_t station_id) {
    boost::shared_lock<boost::shared_mutex> lock(mutex_cur_map_);
    if (!cur_navigation_map_) {
        LOG(WARNING) << "地图未加载！！！";
        return sros::map::StationMark();
    }

    sros::map::StationMarkGroup *station_group = cur_navigation_map_->getStationMarkGroup();
    return station_group->getItem(station_id);
}

std::vector<sros::map::StationMark> MapManager::getStationList() {
    boost::shared_lock<boost::shared_mutex> lock(mutex_cur_map_);
    if (!cur_navigation_map_) {
        LOG(WARNING) << "地图未加载！！！";
        return std::vector<sros::map::StationMark>();
    }

    return cur_navigation_map_->getStationMarkGroup()->getItemList();
}

sros::map::DMCodeMark MapManager::getDMCode(const std::string &id) {
    boost::shared_lock<boost::shared_mutex> lock(mutex_cur_map_);
    if (!cur_navigation_map_) {
        LOG(WARNING) << "地图未加载！！！";
        return sros::map::DMCodeMark();
    }

    auto *dmcode_group = cur_navigation_map_->getDMCodeMarkGroup();
    return dmcode_group->getItem(id);
}
std::vector<sros::map::AreaMark> MapManager::getInsideArea(double x, double y, uint16_t area_type) {
    boost::shared_lock<boost::shared_mutex> lock(mutex_cur_map_);
    if (!cur_navigation_map_) {
        LOG(WARNING) << "地图未加载！！！";
        return std::vector<sros::map::AreaMark>();
    }

    return cur_navigation_map_->getAreaMarkGroup()->getInsideMarks(x, y, area_type);
}

string getGrayMapPath(const std::string &map_name) { return getGrayMapPath(map_name, 0); }

std::string getGrayMapPath(const std::string &map_name, int level) {
    return MapManager::MAP_SAVE_PATH + map_name + std::to_string(level) + ".pgm";
}

string getNavigationMapPath(const std::string &map_name) { return MapManager::MAP_SAVE_PATH + map_name + ".map"; }

std::vector<std::string> getMapFilesPath(const std::string &map_name) { return getOptimizeMapFilesPath(map_name); }

std::vector<std::string> getNavMapFilesPath(const std::string &map_name) {
    std::vector<std::string> files;

    files.push_back(getNavigationMapPath(map_name));

    for (int i = 0; i < 4; i++) {
        files.push_back(getGrayMapPath(map_name, i));
        files.push_back(getGrayMapPath(map_name + "_1", i));
        files.push_back(getGrayMapPath(map_name + "_update", i));
        files.push_back(getGrayMapPath(map_name + "_1_update", i));
    }
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".lom");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".region");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".json");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".png");

    // 不同分辨率的图片
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + "_2.png");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + "_4.png");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + "_8.png");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".tiff");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + "_update.tiff");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".info");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".tiff");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + "_update.tiff");

    return files;
}

std::vector<std::string> getOptimizeMapFilesPath(const std::string &map_name) {
    // NAV + Optimize
    auto files = getNavMapFilesPath(map_name);

    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".om");
    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".g2o");

    //    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".scan");

    return files;
}

std::vector<std::string> getAllMapFilesPath(const std::string &map_name) {
    // NAV + Optimize + .bag
    auto files = getOptimizeMapFilesPath(map_name);

    files.push_back(MapManager::MAP_SAVE_PATH + map_name + ".bag");

    return files;
}

}  // namespace core
}  // namespace sros
