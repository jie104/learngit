//
// Created by lhx on 16-11-22.
//

#ifndef SROS_NET_NAV_H
#define SROS_NET_NAV_H

#include <string>
#include "core/pose.h"
#include "core/navigation_path.h"
#include "core/map/NavigationMap.hpp"

class NetNavigation {

public:

    /**
     * @brief 设定参数
     * @param name 参数名
     * @param value 参数值
     */
    bool setParam(std::string name, std::string value);

    /**
     * @brief 载入地图信息
     * 不加载灰度数据
     *
     * @param file_path 文件路径
     */
    bool loadMapFile(const std::string file_path);

    sros::core::NavigationPath_vector getNavigationPaths(sros::core::StationNo_t start_station_no,
                                                         sros::core::StationNo_t dst_station_no,
                                                         sros::core::Pose &out_dst_pose);

    bool checkIsOnNet(sros::core::StationNo_t station_no, sros::core::Pose pose);

    /**
     * 获取距离给定Pose最近的一个Edge
     * @param pose
     * @return
     */
    sros::map::net::Edgef getNearestEdgeOnNet(sros::core::Pose pose);

private:

    //导航地图
    sros::map::NavigationMap_ptr navigation_map_;
};


#endif //SROS_NET_NAV_H
