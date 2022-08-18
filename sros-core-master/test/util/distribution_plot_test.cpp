/**
 * @file distribution_plot_test
 *
 * @author pengjiali
 * @date 2020/9/15.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <iostream>
#include "../../core/util/distribution_plot.h"

using namespace std;

TEST(a, a) {
    DistributionPlot plot("测试", 10);
    plot.shooting(160);
    plot.shooting(160);
    plot.shooting(60);
    plot.shooting(100);
    std::cout << plot;
}