/**
 * @file Logger.h
 *
 * @author pengjiali
 * @date 19-7-23.
 *
 * @describe 日志类，用于提取可以给用户读取的日志
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_CORE_LOGGER_H
#define SROS_CORE_LOGGER_H

#include <glog/logging.h>
#include <string>
#include <strstream>
#include "user_log.h"

#if GOOGLE_STRIP_LOG == 0
#define COMPACT_STD_LOG_INFO(key) Logger(key, __FILE__, __LINE__)
#else
#define COMPACT_STD_LOG_INFO google::NullStream()
#endif

#if GOOGLE_STRIP_LOG <= 1
#define COMPACT_STD_LOG_WARNING(key) Logger(key, __FILE__, __LINE__, google::GLOG_WARNING)
#else
#define COMPACT_STD_LOG_WARNING google::NullStream()
#endif

#if GOOGLE_STRIP_LOG <= 2
#define COMPACT_STD_LOG_ERROR(key) Logger(key, __FILE__, __LINE__, google::GLOG_ERROR)
#else
#define COMPACT_STD_LOG_ERROR google::NullStream()
#endif

#if GOOGLE_STRIP_LOG <= 3
#define COMPACT_STD_LOG_FATAL(key) LoggerFatal(key, __FILE__, __LINE__)
#else
#define COMPACT_STD_LOG_FATAL google::NullStreamFatal()
#endif

#define LOGGER(severity, key) COMPACT_STD_LOG_##severity(key).stream()

// NOTE: 新增一种日志类型时需要在code.py的API_ClassicUserLogFollow中添加上传的模块UserLogModules
const std::string CMD_HANDER = "command-hander";    // 处理命令时输出的日志
const std::string PROTOBUF = "protobuf";            // protobuf通信
const std::string MODBUS = "modbus";                // modbus通信
const std::string MOVEMENT_TASK = "movement-task";  // 移动任务相关日志
const std::string ACTION_TASK = "action-task";
const std::string EXCUTE_ERROR = "exec-error";  // 执行出错
const std::string SROS = "sros";
const std::string TASK = "task";  // task模块
const std::string DEVICE = "device";
const std::string SCHEDULE = "schedule";

class Logger {
 public:
    Logger(const std::string& key, const char* file, int line) : logger_(file, line), key_(key) {}
    Logger(const std::string& key, const char* file, int line, int severity)
        : logger_(file, line, severity), key_(key), severity_(severity) {}
    ~Logger() {
        logger_.stream() << "{" << key_ << "} " << stream_.str();
        logger_.Flush();
        if (inited_) {
            UserLog::getInstance().write(severity_, key_, stream_.str());
        }
    }

    static void init() { inited_ = true; }

    std::ostream& stream() { return stream_; }

 private:
    std::stringstream stream_;
    google::LogMessage logger_;
    const std::string& key_;
    int severity_ = google::GLOG_INFO;
    static bool inited_;  // 当没有初始化就不要不要输出到用户日志了，方便测试
};

class LoggerFatal {
 public:
    LoggerFatal(const std::string& key, const char* file, int line) : logger_fault_(file, line), key_(key) {}
    ~LoggerFatal() {
        logger_fault_.stream() << "{" << key_ << "} " << stream_.str();
        logger_fault_.Flush();
        //        UserLog::getInstance().write(google::GLOG_FATAL,  key_, stream_.str()); //
        //        直接退出了也不需要输出到用户日志
    }

    std::ostream& stream() { return stream_; }

 private:
    std::stringstream stream_;
    google::LogMessageFatal logger_fault_;
    const std::string& key_;
};

#endif  // SROS_CORE_LOGGER_H
