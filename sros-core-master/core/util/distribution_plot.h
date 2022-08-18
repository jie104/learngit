/**
 * @file distribution_plot
 *
 * @author pengjiali
 * @date 2020/9/11.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_DISTRIBUTION_PLOT_H
#define SROS_DISTRIBUTION_PLOT_H

#include <map>
#include <ostream>
#include <vector>

/**
 * 记录数据分布特性
 * 暂时不是线程安全的
 */
class DistributionPlot {
 public:
    DistributionPlot(const std::string& name, int distribution_segment_width = 10);
    void shooting(int value);

    int shoot_count() const { return current_shoot_count_; }

    friend std::ostream& operator<<(std::ostream& out, const DistributionPlot& plot);

    friend void calcSdValue(const DistributionPlot& plot, double& average, double& sd_value);

 private:
    int current_shoot_count_ = 0;          // 当前记录的次数
    std::map<int, int> distribution_map_;  // 数据分布图，<数据段，落在该段的数据个数>
    int distribution_segment_width_ = 10;  // 数据分配宽度
    const std::string name_;               // 分布图的名字
};

#endif  // SROS_DISTRIBUTION_PLOT_H
