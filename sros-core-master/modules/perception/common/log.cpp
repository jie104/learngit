/**
 * @file log.cpp
 * @brief log class
 *
 * The log class encapsulates the interface functions required by the glog system,
 * if you want to use，If you want to use the log system, you first need to refer
 * to the log.h header file and then call the init function.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */


// INCLUDE
#include "log.h"

#include <mutex>
#include <string>
#include <glog/logging.h>
#include "common_func.hpp"
//#include "common_type.hpp"

// CODE
std::once_flag Logging::init_;
std::string Logging::log_path_;
std::string Logging::file_name_;

void Logging::init(const std::string &log_path,
                   const std::string &file_name)
{
    Logging::log_path_ = log_path;
    Logging::file_name_ = file_name;
    std::call_once(Logging::init_, Logging::_init);
}

void Logging::_init()
{
    //初始化参数
    FLAGS_logtostderr = false;          //TRUE:标准输出,FALSE:文件输出
    FLAGS_alsologtostderr = true;       //除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = false;     //标准输出带颜色
    FLAGS_logbufsecs = 0;               //设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 10;            //日志文件大小(单位：MB)
    FLAGS_stop_logging_if_full_disk = true; //磁盘满时记录到磁盘
    FLAGS_logbuflevel = -1;
    FLAGS_log_dir = log_path_;
    std::string file = common_func::getDateTimeStr() + ".log";
    google::InitGoogleLogging(file_name_.c_str());
    google::SetStderrLogging(google::INFO);    //设置日志级别
//    google::SetStderrLogging(google::WARNING);    //设置日志级别
}

void Logging::shutdownLog(){
    google::ShutdownGoogleLogging();
}
