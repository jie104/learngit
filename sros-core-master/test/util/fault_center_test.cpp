/**
 * @file fault_center_test
 *
 * @author pengjiali
 * @date 2021/4/23.
 *
 * @describe
 *
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <chrono>
#include <ctime>
#include <future>
#include <thread>
#include "core/fault_center.h"
#include "core/util/distribution_plot.h"
#include "core/util/time.h"

using namespace sros::core;

FaultCenter* fault_center = nullptr;
bool observer_finish = false;
std::map<int, std::pair<double, double>> observer_count_plot_map;  // 不同观察者个数对应的平均数和方差

/**
 * 故障检测对象，随机触发以故障，但也由此线程关闭
 */
void faultChecker(int fault_code, int fault_duration_ms) {
    for (int n = 0; n < 1e10 && !observer_finish; ++n) {
        fault_center->addFault(fault_code);
        std::this_thread::sleep_for(std::chrono::milliseconds(fault_duration_ms));
        fault_center->removeFault(fault_code);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * 故障观察者，要实时获取故障，且故障经过排序
 */
void observer(bool output_plot, int observer_count) {
    DistributionPlot plot("observer读取时间us", 100);
    for (auto i = 0; i < 1e7; ++i) {
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto start = util::get_time_in_ns();
        auto list = fault_center->getFaultList();
        auto end = util::get_time_in_ns();
        if (!list->empty()) {
            list->front()->raise_timestamp += 1;
            if (list->front()->raise_timestamp == 0) {
                std::cout << list->front()->id << std::endl;
            }
        }
        plot.shooting(end - start);
    }

    if (output_plot) {
        std::cout << plot << std::endl;
        double average;
        double sd_value;
        calcSdValue(plot, average, sd_value);
        observer_count_plot_map[observer_count] = std::make_pair(average, sd_value);
    }
}

TEST(FaultCenter, All) {
    fault_center = FaultCenter::getInstance();
    std::thread faultChecker_1(faultChecker, 212438, 10);
    std::thread faultChecker_2(faultChecker, 211438, 10);
    std::thread faultChecker_3(faultChecker, 11323, 10);
//    std::thread faultChecker_4(faultChecker, 711403, 4);
//    std::thread faultChecker_5(faultChecker, 621499, 5);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int observer_num = 1;
    for (auto timer = 1; timer <= observer_num; ++timer) {
        std::thread observer_1(observer, true, timer);
        std::vector<std::shared_ptr<std::thread>> ts;
        for (auto i = 0; i < timer - 1; ++i) {
            std::shared_ptr<std::thread> t = std::make_shared<std::thread>(observer, false, timer);
            ts.push_back(t);
        }

        observer_1.join();
        for (auto t : ts) {
            t->join();
        }
    }

    observer_finish = true;
    faultChecker_1.join();
    faultChecker_2.join();
    faultChecker_3.join();
//    faultChecker_4.join();
//    faultChecker_5.join();

    for (auto it=observer_count_plot_map.cbegin(); it!=observer_count_plot_map.cend(); ++it) {
        std::cout << it->first << "(" << it->second.first << ", " << it->second.second << ") ";
    }
    std::cout << std::endl;
}