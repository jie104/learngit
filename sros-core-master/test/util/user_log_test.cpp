/**
 * @file user_log_test
 *
 * @author pengjiali
 * @date 2021/3/2.
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
#include "core/logger.h"
#include "core/util/distribution_plot.h"
#include "core/util/time.h"

/**
 * 用单线程写数，测试性能：
 * v4.12.0版本新能如下：
 * total count: 2000000,   segment width: 1,   average: 1.0518,   sd_value: 0.709222
*         0 ~          0   [97%]
|█████████████████████████████████████████████████████████████████████████████████████████████████ 1952661 1 ~ 1   [ 2%]
|██ 43306 2 ~          2   [ 0%] | 8 3 ~          3   [ 0%] | 8 4 ~          4   [ 0%] | 12 5 ~          5   [ 0%] | 6
          6 ~          6   [ 0%] | 1
          7 ~          7   [ 0%] | 1
          8 ~          8   [ 0%] | 0
          9 ~          9   [ 0%] | 0
         10 ~         10   [ 0%] | 33
         11 ~         11   [ 0%] | 442
         12 ~         12   [ 0%] | 632
         13 ~         13   [ 0%] | 66
         14 ~         14   [ 0%] | 12
         15 ~         15   [ 0%] | 3
         16 ~         16   [ 0%] | 7
         17 ~         17   [ 0%] | 4
         18 ~         18   [ 0%] | 607
         19 ~         19   [ 0%] | 1606
         20 ~         20   [ 0%] | 211
         21 ~         21   [ 0%] | 246
         22 ~         22   [ 0%] | 63
         23 ~         23   [ 0%] | 9

         24 ~         24   [ 0%] | 14
         25 ~         25   [ 0%] | 10
         26 ~         26   [ 0%] | 4
         27 ~         27   [ 0%] | 1
         28 ~         28   [ 0%] | 0
         29 ~         29   [ 0%] | 0
         30 ~         30   [ 0%] | 0
......
         31 ~         31   [ 0%] | 2
         32 ~         32   [ 0%] | 2
         33 ~         33   [ 0%] | 1
         34 ~         34   [ 0%] | 1
         35 ~         35   [ 0%] | 4
         36 ~         36   [ 0%] | 2
         37 ~         37   [ 0%] | 1
         38 ~         38   [ 0%] | 0
         39 ~         39   [ 0%] | 0
         40 ~         40   [ 0%] | 1
         41 ~         41   [ 0%] | 1
         42 ~         42   [ 0%] | 0
         43 ~         43   [ 0%] | 1
         44 ~         44   [ 0%] | 0
         45 ~         45   [ 0%] | 1
         46 ~         46   [ 0%] | 1
         47 ~         47   [ 0%] | 0
         48 ~         48   [ 0%] | 0
         49 ~         49   [ 0%] | 0
......
         50 ~         50   [ 0%] | 1
         51 ~         51   [ 0%] | 0
         52 ~         52   [ 0%] | 0
         53 ~         53   [ 0%] | 0
......
         54 ~         54   [ 0%] | 1
         55 ~         55   [ 0%] | 0
         56 ~         56   [ 0%] | 1
         57 ~         57   [ 0%] | 0
         58 ~         58   [ 0%] | 1
         59 ~         59   [ 0%] | 0
         60 ~         60   [ 0%] | 0
         61 ~         61   [ 0%] | 0
......
         74 ~         74   [ 0%] | 1
         75 ~         75   [ 0%] | 0
         76 ~         76   [ 0%] | 0
         77 ~         77   [ 0%] | 0
......
        270 ~        270   [ 0%] | 1
        271 ~        271   [ 0%] | 0
        272 ~        272   [ 0%] | 0
        273 ~        273   [ 0%] | 0
......
        279 ~        279   [ 0%] | 1
        280 ~        280   [ 0%] | 0
        281 ~        281   [ 0%] | 0
        282 ~        282   [ 0%] | 0
......
        288 ~        288   [ 0%] | 1
        289 ~        289   [ 0%] | 0
        290 ~        290   [ 0%] | 0
        291 ~        291   [ 0%] | 0
......
        296 ~        296   [ 0%] | 1
Process finished with exit code 0

 v4.13.0:
 total count: 2000000,   segment width: 1,   average: 1.02279,   sd_value: 0.164977
*         0 ~          0   [97%] |█████████████████████████████████████████████████████████████████████████████████████████████████ 1955594
          1 ~          1   [ 2%] |██ 44145
          2 ~          2   [ 0%] | 61
          3 ~          3   [ 0%] | 47
          4 ~          4   [ 0%] | 29
          5 ~          5   [ 0%] | 23
          6 ~          6   [ 0%] | 14
          7 ~          7   [ 0%] | 23
          8 ~          8   [ 0%] | 19
          9 ~          9   [ 0%] | 13
         10 ~         10   [ 0%] | 7
         11 ~         11   [ 0%] | 5
         12 ~         12   [ 0%] | 6
         13 ~         13   [ 0%] | 3
         14 ~         14   [ 0%] | 1
         15 ~         15   [ 0%] | 3
         16 ~         16   [ 0%] | 0
         17 ~         17   [ 0%] | 2
         18 ~         18   [ 0%] | 0
         19 ~         19   [ 0%] | 2
         20 ~         20   [ 0%] | 0
         21 ~         21   [ 0%] | 0
         22 ~         22   [ 0%] | 1
         23 ~         23   [ 0%] | 0
         24 ~         24   [ 0%] | 0
         25 ~         25   [ 0%] | 0
......
         29 ~         29   [ 0%] | 1
         30 ~         30   [ 0%] | 0
         31 ~         31   [ 0%] | 0
         32 ~         32   [ 0%] | 0
......
         36 ~         36   [ 0%] | 1
user_log_test: /home/john/workspace/sros-core/thirty-party/SQLiteCpp/src/Database.cpp:104: SQLite::Database::~Database(): Assertion `0 == ret && "database is locked"' failed.
Process finished with exit code 134 (interrupted by signal 6: SIGABRT)


 单独glog：
 total count: 2000000,   segment width: 1,   average: 1.01074,   sd_value: 0.126863
*         0 ~          0   [99%]
|███████████████████████████████████████████████████████████████████████████████████████████████████ 1980199 1 ~ 1   [
0%] | 19335 2 ~          2   [ 0%] | 199 3 ~          3   [ 0%] | 8 4 ~          4   [ 0%] | 10 5 ~          5   [ 0%] |
49 6 ~          6   [ 0%] | 94 7 ~          7   [ 0%] | 55 8 ~          8   [ 0%] | 21 9 ~          9   [ 0%] | 16 10 ~
10   [ 0%] | 3 11 ~         11   [ 0%] | 4 12 ~         12   [ 0%] | 1 13 ~         13   [ 0%] | 0 14 ~         14   [
0%] | 3 15 ~         15   [ 0%] | 1 16 ~         16   [ 0%] | 1 17 ~         17   [ 0%] | 1 Process finished with exit
code 0 单独用user_log: total count: 2000000,   segment width: 1,   average: 1.02107,   sd_value: 0.143897
*         0 ~          0   [97%]
|█████████████████████████████████████████████████████████████████████████████████████████████████ 1957896 1 ~ 1   [ 2%]
|██ 42097 2 ~          2   [ 0%] | 1 3 ~          3   [ 0%] | 1 4 ~          4   [ 0%] | 0 5 ~          5   [ 0%] | 0 6
~          6   [ 0%] | 1 7 ~          7   [ 0%] | 0 8 ~          8   [ 0%] | 0 9 ~          9   [ 0%] | 0
......
         10 ~         10   [ 0%] | 1
         11 ~         11   [ 0%] | 1
         12 ~         12   [ 0%] | 0
         13 ~         13   [ 0%] | 1
         14 ~         14   [ 0%] | 0
         15 ~         15   [ 0%] | 0
         16 ~         16   [ 0%] | 0
......
         38 ~         38   [ 0%] | 1

Process finished with exit code 0
 */
TEST(UserLog, OneThread) {
    //    FLAGS_log_dir = "/sros/log";
    //    google::InitGoogleLogging("aaaaaaaa");
    //    FLAGS_colorlogtostderr = true;
    //    FLAGS_minloglevel = 0;
    //    FLAGS_logbufsecs = 0;
    //
    //    // 警告日志和错误日志现在都没人看，都是INFO的日志，有时候还有人误吧警告日志当成INFO日志，所有先去除。
    //    google::SetLogDestination(google::WARNING, "");
    //    google::SetLogDestination(google::ERROR, "");
    //
    //    google::SetStderrLogging(google::INFO);
    //    google::InstallFailureSignalHandler();

    UserLog::getInstance().set_lines_pre_file(1000);
    static DistributionPlot write_user_log_interval_plot("写一次日志需要的时间(ms)", 1);
    for (auto i = 0; i < 2e6; ++i) {
        auto time_begin = sros::core::util::get_time_in_ms();
        LOGGER(INFO, SROS) << "User log perform test!!! " << i;
        auto time_end = sros::core::util::get_time_in_ms();
        write_user_log_interval_plot.shooting(time_end - time_begin);
    }
    LOG(INFO) << write_user_log_interval_plot;
}

/**
 * 用两个线程写数，测试性能：
 * v4.12.0版本新能如下：
total count: 1000000,   segment width: 1,   average: 1.03284,   sd_value: 0.19636
*         0 ~          0   [96%]
|████████████████████████████████████████████████████████████████████████████████████████████████ 968237 1 ~          1
[ 3%] |███ 31503 2 ~          2   [ 0%] | 34 3 ~          3   [ 0%] | 22 4 ~          4   [ 0%] | 33 5 ~          5   [
0%] | 65 6 ~          6   [ 0%] | 56 7 ~          7   [ 0%] | 28 8 ~          8   [ 0%] | 11 9 ~          9   [ 0%] | 3
         10 ~         10   [ 0%] | 2
         11 ~         11   [ 0%] | 1
         12 ~         12   [ 0%] | 1
         13 ~         13   [ 0%] | 1
         14 ~         14   [ 0%] | 1
         15 ~         15   [ 0%] | 0
         16 ~         16   [ 0%] | 0
         17 ~         17   [ 0%] | 1
         18 ~         18   [ 0%] | 0
         19 ~         19   [ 0%] | 0
         20 ~         20   [ 0%] | 1
Process finished with exit code 0

 v 4.13.0
 total count: 1000000,   segment width: 1,   average: 1.0481,   sd_value: 0.40657
*         0 ~          0   [96%]
|████████████████████████████████████████████████████████████████████████████████████████████████ 963475 1 ~          1
[ 3%] |███ 34034 2 ~          2   [ 0%] | 752 3 ~          3   [ 0%] | 379 4 ~          4   [ 0%] | 264 5 ~          5
[ 0%] | 232 6 ~          6   [ 0%] | 140 7 ~          7   [ 0%] | 99 8 ~          8   [ 0%] | 102 9 ~          9   [ 0%]
| 97 10 ~         10   [ 0%] | 75 11 ~         11   [ 0%] | 66 12 ~         12   [ 0%] | 78 13 ~         13   [ 0%] | 40
         14 ~         14   [ 0%] | 35
         15 ~         15   [ 0%] | 21
         16 ~         16   [ 0%] | 28
         17 ~         17   [ 0%] | 16
         18 ~         18   [ 0%] | 14
         19 ~         19   [ 0%] | 2
         20 ~         20   [ 0%] | 8
         21 ~         21   [ 0%] | 9
         22 ~         22   [ 0%] | 6
         23 ~         23   [ 0%] | 5
         24 ~         24   [ 0%] | 5
         25 ~         25   [ 0%] | 1
         26 ~         26   [ 0%] | 5
         27 ~         27   [ 0%] | 1
         28 ~         28   [ 0%] | 1
         29 ~         29   [ 0%] | 2
         30 ~         30   [ 0%] | 2
         31 ~         31   [ 0%] | 2
         32 ~         32   [ 0%] | 0
         33 ~         33   [ 0%] | 2
         34 ~         34   [ 0%] | 2
Process finished with exit code 0


 */
TEST(UserLog, TwoThread) {
    UserLog::getInstance().set_lines_pre_file(1e7);
    static DistributionPlot write_user_log_interval_plot("写一次日志需要的时间(ms)", 1);
    auto fun1 = [&]() {
        for (auto i = 0; i < 1e6; ++i) {
            auto time_begin = sros::core::util::get_time_in_ms();
            LOGGER(INFO, SROS) << "thread 1: User log perform test!!! " << i;
            auto time_end = sros::core::util::get_time_in_ms();
            write_user_log_interval_plot.shooting(time_end - time_begin);
        }
    };
    auto fun2 = []() {
        for (auto i = 0; i < 1.05e6; ++i) {
            LOGGER(INFO, SROS) << "thread 2: User log perform test!!! " << i;
        }
    };
    std::thread t1(fun1);
    std::thread t2(fun2);
    t1.join();
    t2.join();
    LOG(INFO) << write_user_log_interval_plot;
}
