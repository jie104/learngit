//
// Created by lhx on 16-1-6.
//

#ifndef SROS_MAP_MANAGER_H
#define SROS_MAP_MANAGER_H

#include <boost/thread/shared_mutex.hpp>
#include <map>
#include <string>
#include "./map/NavigationMap.hpp"
#include "./map/mark/StationMark.hpp"
#include "map/mark/DMCodeMark.hpp"
#include "map_info.hpp"

namespace sros {
namespace core {

typedef std::map<std::string, MapInfo> MapInfoMap_t;

class MapManager {
 public:
    MapManager();
    virtual ~MapManager();

    static MapManager *getInstance();

    void freshMapList();

    std::vector<std::string> getMapListSortByName() const;

    MapVector_t getMapInfoList() const;
    MapInfo getMapInfo(const std::string &map_name) const;

    sros::map::NavigationMap_ptr getNavigationMap() const { return cur_navigation_map_; }

    static std::string MAP_SAVE_PATH;
    static void setMapSavePath(const std::string &path) { MAP_SAVE_PATH = path; }  // NOTE：调试专用

    // 当被地图更新后需要调用此函数。（如：HTTP修改了某个地图后，需要调用此函数，不然还会用以前缓存的老地图）
    void mapUpdated(const std::string &map_name);

    bool isLoadMap() const { return is_load_map_; }
    bool setCurrentMap(const std::string &map_name);  // 设置当前地图

    sros::map::MapFileVersion getMapVersion() { return cur_navigation_map_->getMapVersion(); }

    sros::map::StationMark getStation(uint16_t station_id); // 获取当前站点信息
    std::vector<sros::map::StationMark> getStationList(); // 获取站点列表

    sros::map::DMCodeMark getDMCode(const std::string &id); // 获取二维码信息

    /**
     * 获取点p（x,y)所在当前的地图中的区域
     * @param x
     * @param y
     * @param area_type 0 -- 获取所有区域类型 ，否则获取指定区域类型
     * @return
     */
    std::vector<sros::map::AreaMark> getInsideArea(double x, double y, uint16_t area_type = 0);

 private:
    mutable boost::shared_mutex mutex_map_list_;  // 当前地图的锁
    MapVector_t map_list_;
    MapInfoMap_t map_infos_;

    //导航地图
    bool is_load_map_ = false;                         // 是否加载了地图
    sros::map::NavigationMap_ptr cur_navigation_map_;  // 当前的导航地图
    mutable boost::shared_mutex mutex_cur_map_;        // 当前地图的锁
};

std::string getGrayMapPath(const std::string &map_name);

std::string getGrayMapPath(const std::string &map_name, int level);

std::vector<std::string> getMapFilesPath(const std::string &map_name);  // NAV

std::vector<std::string> getNavMapFilesPath(const std::string &map_name);  // NAV + Optimize

std::vector<std::string> getOptimizeMapFilesPath(const std::string &map_name);  // NAV + Optimize

std::vector<std::string> getAllMapFilesPath(const std::string &map_name);  // NAV + Optimize + .bag

std::string getNavigationMapPath(const std::string &map_name);

}  // namespace core
}  // namespace sros

#endif  // SROS_MAP_MANAGER_H
