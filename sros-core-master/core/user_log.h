/**
 * @file user_log.h
 *
 * @author pengjiali
 * @date 19-8-22.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef USER_LOG_H
#define USER_LOG_H

#include <glog/logging.h>
#include <glog/log_severity.h>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include "core/util/json.h"

class UserLog {
 public:
    static UserLog &getInstance();

    /**
     * @brief 日志输出
     * @param severity
     * @param str 要输出的内容，不能带'\n'符号
     */
    void write(google::LogSeverity severity, const std::string &key, const std::string &str);

    /**
     * 设置一个文件多少行
     * @param num
     */
    void set_lines_pre_file(uint32_t num) { LINES_PER_FILE = num; }

 private:
    UserLog();
    void writer();

    bool enable_ = false;  // 是否使能
    std::fstream fstream_;
    uint32_t curr_log_id_ = 0;
    uint32_t curr_file_id_ = 0;       // 当前日志文件id
    uint32_t curr_line_ = 0;          // 当前文件的行数
    uint32_t LINES_PER_FILE = 10000;  // 每个文件多少行，大概是一万行为1M
    std::mutex mutex_;
    std::condition_variable condition_fill_;
    std::queue<nlohmann::json> buffer_;  // 用于缓存日志
};

#endif  // USER_LOG_H
