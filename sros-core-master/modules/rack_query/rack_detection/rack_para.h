//
// Created by lfc on 18-12-19.
//

#ifndef SROS_RACK_PARA_H
#define SROS_RACK_PARA_H
struct RackLegGroup{
    double length = 0.0;//每个货架腿组包含四个腿，分布在矩形的四个角，因此，只需获取length与width就能计算出所有腿对应的位置与尺寸
    double width = 0.0;
};

struct RackInfo {
    double leg_d = 0.05;
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
#endif //SROS_RACK_PARA_H
