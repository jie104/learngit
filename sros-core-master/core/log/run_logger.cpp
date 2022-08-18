/**
 * @file run_logger.cpp
 *
 * @author lhx
 * @date 2018年4月25日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "run_logger.h"

#include <glog/logging.h>

#include "core/util/timer.h"

namespace sros {
namespace core {

const char* TABLE_NAME = "run_log";

RunLogger& RunLogger::getInstance() {
    static RunLogger run_logger;
    return run_logger;
}

RunLogger::RunLogger() : id_(0), mileage_zero_offset_(0), db_(&g_db) {}

RunLogger::RunLogger(RunLogger const&)
    : id_(0), mileage_zero_offset_(0), db_(&g_db) {}

RunLogger& RunLogger::operator=(RunLogger const&) { return *this; }

RunLogger::~RunLogger() {}

int RunLogger::newRun(const std::string& sros_version_str, const std::string src_version_str) {
    if (id_ != 0) {
        LOG(WARNING) << "RunLogger -> id = " << id_;
        return 0;
    }

    auto cur_timestamp = util::get_timestamp_in_s();
    std::string sql = std::string("") + " INSERT INTO " + TABLE_NAME + " VALUES (" + "NULL, " +
                      std::to_string(cur_timestamp) + " , " + std::to_string(cur_timestamp) + " , " + "0, " + "'" +
                      sros_version_str + "' , " + "'" + src_version_str + "'" + ")";

    LOG(INFO) << "SQL: " << sql;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        if (db_->exec(sql)) {
            id_ = static_cast<int>(db_->getLastInsertRowid());
        }
    } catch (std::exception& e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    return id_;
}

uint32_t RunLogger::keepAlive(uint32_t mileage) {
    if (id_ == 0) {
        return 0;
    }

    auto real_mileage = mileage - mileage_zero_offset_;
    if (real_mileage < 0) {
        LOG(ERROR) << "Real mileage less then 0!!!, mileage:" << mileage
                   << " mileage_zero_offset_:" << mileage_zero_offset_;
        real_mileage = 0;
    }

    auto cur_timestamp = util::get_timestamp_in_s();
    std::string sql = std::string("") + " UPDATE " + TABLE_NAME + " SET " +
                      "last_alive_time = " + std::to_string(cur_timestamp) + " , " +
                      "move_mileage = " + std::to_string(real_mileage) + " " + " WHERE id = " + std::to_string(id_);

    //    LOG(INFO) << "SQL: " << sql;
    cache_.erase("total_run_time");
    cache_.erase("total_mileage");

    int r = 0;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        r = db_->exec(sql);
    } catch (std::exception& e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    return r;
}

int RunLogger::getTotalRunTimeInSeconds() {
    auto iter = cache_.find("total_run_time");
    if (iter != cache_.end()) {
        return cache_["total_run_time"];
    }

    std::string sql = "SELECT start_time, last_alive_time FROM " + std::string(TABLE_NAME);

    int total_run_time = 0;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Statement query(*db_, sql);

        while (query.executeStep()) {
            auto item_boot_time = query.getColumn(0).getInt();
            auto item_last_alive_time = query.getColumn(1).getInt();

            auto run_time = item_last_alive_time - item_boot_time;
            if (run_time > 0) {
                total_run_time += run_time;
            }
        }
    } catch (std::exception& e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    cache_["total_run_time"] = total_run_time;
    return total_run_time;
}

int RunLogger::getTotalMileage() {
    auto iter = cache_.find("total_mileage");
    if (iter != cache_.end()) {
        return cache_["total_mileage"];
    }

    // NOTE: 测试过虽然move_mileage字段是字符串，但是还是可以用' > 0' 来判断
    std::string sql = "SELECT SUM(move_mileage) FROM " + std::string(TABLE_NAME) + " WHERE move_mileage > 0";

    int total_mileage = 0;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        total_mileage = db_->execAndGet(sql).getInt();
    } catch (std::exception& e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    cache_["total_mileage"] = total_mileage;

    return total_mileage;
}

int RunLogger::getTotalBootTimes() {
    auto iter = cache_.find("total_boot_times");
    if (iter != cache_.end()) {
        return cache_["total_boot_times"];
    }

    std::string sql = "SELECT COUNT(*) FROM " + std::string(TABLE_NAME);

    int total_boot_times = 0;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        total_boot_times = db_->execAndGet(sql).getInt();
    } catch (std::exception& e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    cache_["total_boot_times"] = total_boot_times;

    return total_boot_times;
}

void RunLogger::setMileageOffset(uint32_t zero_offset) { mileage_zero_offset_ = zero_offset; }

}  // namespace core
}  // namespace sros
