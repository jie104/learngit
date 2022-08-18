//
// Created by lfc on 19-1-17.
//

#ifndef PROJECT_RACK_PARA_HPP
#define PROJECT_RACK_PARA_HPP

#include <memory>

namespace rack {
struct RackPara {
    double rack_width = 0.8;
    double rack_length = 1.35;
    double rack_leg_radius = 0.05;//这里，半径是直径的意思，后期需要修改
};


struct InstallPara{
    double laser_angle_min = -2.2;
    double laser_angle_max = 2.2;
    double laser_coord_x = 0.345;
    double laser_coord_y = 0.0;
    double laser_coord_yaw = 0.0;
    double backlash_angle = 0.034;
};

struct RackLegGroup{
    double length = 0.0;//每个货架腿组包含四个腿，分布在矩形的四个角，因此，只需获取length与width就能计算出所有腿对应的位置与尺寸
    double width = 0.0;
};

struct RackInfo {
    double avd_oba_length = 1.2;
    double avd_oba_width = 0.6;
    double leg_d = 0.05;
    RackPara rack_para;
    std::vector<RackLegGroup> leg_groups;//并非每个货架只有四个腿,部分有12腿.

    bool operator<(const RackInfo &rhs) const {
        const auto &size = leg_groups.front();
        const auto &rsize = rhs.leg_groups.front();
        if (size.length == rsize.length) {
            return size.width < rsize.width;
        }
        return size.length < rsize.length;
    }
};

typedef std::shared_ptr<RackInfo> RackInfo_Ptr;
}


#endif //PROJECT_RACK_PARA_HPP
