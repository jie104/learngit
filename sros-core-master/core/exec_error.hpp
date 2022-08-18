/**
 * @file ExecError.hpp
 *
 * @author pengjiali
 * @date 19-8-7.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef EXEC_ERROR_HPP
#define EXEC_ERROR_HPP

#include <map>
#include <stdexcept>
#include "core/db/db.h"
#include "core/msg/hmi_msg.hpp"
#include "core/msg_bus.h"
#include "core/task/task_manager.h"
#include "logger.h"
#include "state.h"
#include "util/utils.h"

const int ActionErrorStartIndex = 400000;  // 动作错误码起始偏移

// 构造一个执行错误的以异常，用于抛出执行出错
#define EXEC_ERROR(error_code, what_args...) makeExecError(__FILE__, __LINE__, error_code, #error_code, what_args)

// 设置执行出错
#define SET_ERROR(error_code, what_args...) setLastError(__FILE__, __LINE__, error_code, #error_code, what_args)

// 设置movement执行出错
#define SET_MOVEMENT_EXEC_FAILED(error_code, what_args...) \
    setMovementExecError(__FILE__, __LINE__, error_code, #error_code, what_args)

// 设置action执行出错
#define SET_ACTION_EXEC_FAILED(error_code, what_args...) \
    setActionExecError(__FILE__, __LINE__, error_code, #error_code, what_args)

namespace sros {
namespace core {

class ErrorCenter {
 public:
    static ErrorCenter* getInstance() {
        static ErrorCenter instance;
        return &instance;
    }

    void setError(uint32_t error_code) {
        g_state.setError(error_code);
        if (music_map_.find(error_code) != music_map_.end()) {
            auto msg = std::make_shared<sros::core::HmiMsg>();
            msg->command = HMI_COMMAND_SET_ERROR_MUSIC;
            msg->int_0 = music_map_.at(error_code);
            MsgBus::sendMsg(msg);
        }
    }

 private:
    ErrorCenter() { loadErrorCode(); }

    void loadErrorCode() {
        const std::string& sql = "SELECT id, music_id FROM error_code";

        try {
            std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
            SQLite::Statement query(g_db, sql);
            while (query.executeStep()) {
                int id = query.getColumn(0).getInt();
                int music_id = query.getColumn(1).getInt();
                if (music_id != 0) {
                    music_map_.insert(std::make_pair(id, music_id));
                }
            }
        } catch (std::exception& e) {
            LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
        }
    }

    std::map<uint32_t, uint32_t> music_map_;  // <错误码，播放对应音乐的ID>
};

class ExecError : std::runtime_error {
 public:
    ExecError(const char* file, int line, uint32_t error_code, const std::string& what_arg)
        : std::runtime_error(what_arg), error_code_(error_code) {
        ErrorCenter::getInstance()->setError(error_code_);
        Logger(EXCUTE_ERROR, file, line, google::GLOG_WARNING).stream() << what_arg;
    }

    virtual ~ExecError() = default;

    uint32_t errorCode() const { return error_code_; }

 private:
    int error_code_ = ERROR_CODE_UNDEFINED;
};

template <typename... Targs>
ExecError makeExecError(const char* file, int line, uint32_t error_code, const std::string& error_code_str,
                        Targs... what_args) {
    return ExecError(file, line, error_code,
                     std::to_string((int)error_code) + " " + error_code_str + " " + getArgStr(what_args...));
}

template <typename... Targs>
void setLastError(const char* file, int line, uint32_t error_code, const std::string& error_code_str,
                  Targs... what_args) {
    Logger(EXCUTE_ERROR, file, line, google::GLOG_WARNING).stream()
        << error_code << " " << error_code_str << " " << getArgStr(what_args...);
    ErrorCenter::getInstance()->setError(error_code);
}

template <typename... Targs>
void setMovementExecError(const char* file, int line, uint32_t error_code, const std::string& error_code_str,
                          Targs... what_args) {
    Logger(EXCUTE_ERROR, file, line, google::GLOG_WARNING).stream()
        << error_code << " " << error_code_str << " " << getArgStr(what_args...);
    ErrorCenter::getInstance()->setError(error_code);
    TaskManager::getInstance()->setMovementFinishFailed(error_code);
}

/**
 *
 * @tparam Targs
 * @param file
 * @param line
 * @param error_code 错误码可以是原始错误码，也可以是添加了地址偏移的错误码
 * @param error_code_str
 * @param what_args
 */
template <typename... Targs>
void setActionExecError(const char* file, int line, uint32_t error_code, const std::string& error_code_str,
                        Targs... what_args) {
    uint32_t error = error_code;
    if (error_code < ActionErrorStartIndex) {
        error += ActionErrorStartIndex;  // 原始错误码需要添加地址偏移才算sros的错误码
    }

    Logger(EXCUTE_ERROR, file, line, google::GLOG_WARNING).stream()
        << error_code << " " << error_code_str << " " << getArgStr(what_args...);
    ErrorCenter::getInstance()->setError(error);
    TaskManager::getInstance()->setActionFinishFailed(error_code);
}

}  // namespace core
}  // namespace sros

#endif  // EXEC_ERROR_HPP
