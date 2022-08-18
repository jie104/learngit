/**
 * @file user_log.cpp
 *
 * @author pengjiali
 * @date 19-8-22.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "user_log.h"
#include <thread>
#include "core/meta_setting.h"
#include "core/util/utils.h"

using json = nlohmann::json;
using namespace google;
using namespace std;

const char LOG_FILE_DIR[] = "/sros/log/";

UserLog::UserLog() {
    curr_file_id_ = sros::core::MetaSetting::getInstance().getValue("log.file_id", curr_file_id_);
    std::string curr_file_name = LOG_FILE_DIR + std::to_string(curr_file_id_) + ".json";
    LOG(INFO) << "Current user log file name is " << curr_file_name;

    // 删掉大于当前记录json id 的日志，比如数据库被重置了就可能会出现记录的日记比当前的日志id大
    auto cmd = R"delimiter(rm `ls /sros/log/*.json | awk -F '[/.]' '{if($4 > )delimiter" + std::to_string(curr_file_id_) +
               R"delimiter() print $0}'`)delimiter";
    LOG(INFO) << cmd;
    execShell(cmd);

    // 日志文件可能会由于断电导致最后是乱码，此处就是清除乱码
    execShell(R"delimiter(awk '/^\{\"id\":.*\"\}$/' )delimiter" + curr_file_name + " > userlog.tmp && mv userlog.tmp " +
              curr_file_name);
    fstream_.open(curr_file_name,
                  std::fstream::in | std::fstream::out | std::fstream::app);  // 不存在就创建
    enable_ = true;
    std::string cmd_to_get_log_id = "grep -h '' `ls " + curr_file_name + " | tail -n 2` | tail -n 1 | awk -F '[:,]' '{print $2}'";
    auto result = execShell(cmd_to_get_log_id);
    LOG(INFO) << "get last id result is " << result;
    try {
        curr_log_id_ = std::stoi(result);
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        curr_log_id_ = 0;
    }
    LOG(INFO) << "user log id is " << curr_log_id_;

    cmd = "wc -l " + curr_file_name + " | awk '{print $1}'";
    result = execShell(cmd);
    LOG(INFO) << cmd;
    LOG(INFO) << "get last line result is " << result;
    try {
        curr_line_ = std::stoi(result);
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
        curr_line_ = 0;
    }
    LOG(INFO) << "user log current line is " << curr_line_;

    std::thread(&UserLog::writer, this).detach();
}

UserLog& UserLog::getInstance() {
    static UserLog instance;
    return instance;
}

void UserLog::write(google::LogSeverity severity, const std::string& key, const std::string& str) {
    if (!enable_) {
        return;
    }

    try {
        json log;

        std::string severity_str;
        switch (severity) {
            case GLOG_INFO: {
                severity_str = "info";
                break;
            }
            case GLOG_WARNING: {
                severity_str = "warning";
                break;
            }
            case GLOG_ERROR: {
                severity_str = "error";
                break;
            }
            case GLOG_FATAL: {
                severity_str = "fatal";
                break;
            }
            default: {
                severity_str = std::to_string(severity);
                break;
            }
        }

        log["id"] = ++curr_log_id_;
        log["time"] = std::move(getCurrentRFC3339Time());
        log["severity"] = std::move(severity_str);
        log["key"] = key;
        log["msg"] = str;

        std::unique_lock<std::mutex> lk(mutex_);
        if (buffer_.size() == 1000) { // 若超过1000条日志就把老的日志丢弃
            buffer_.pop();
        }
        buffer_.push(std::move(log));
        condition_fill_.notify_one();
    } catch (const std::exception& e) {
        LOG(ERROR) << e.what();
    }
}

void UserLog::writer() {
    while(true) {
        std::unique_lock<std::mutex> lk(mutex_);
        while (buffer_.empty()) {
            condition_fill_.wait(lk);
        }
        auto log = std::move(buffer_.front());
        buffer_.pop();
        lk.unlock();

        // 真正开始写日志
        if (++curr_line_ > LINES_PER_FILE) {
            sros::core::MetaSetting::getInstance().setValue("log.file_id", ++curr_file_id_);
            fstream_.close();
            std::string curr_file_name = LOG_FILE_DIR + std::to_string(curr_file_id_) + ".json";
            LOG(INFO) << "New user log file name is " << curr_file_name;
            fstream_.open(curr_file_name, std::fstream::in | std::fstream::out | std::fstream::app);
            if (!fstream_.is_open()) {
                LOG(ERROR) << "open " << curr_file_name << " error!";
                enable_ = false;
            }
            curr_line_ = 0;
        }

        ++curr_line_;
        fstream_ << log << std::endl;
    }
}
