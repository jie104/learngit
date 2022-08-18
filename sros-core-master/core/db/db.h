/**
 * @file db.h
 *
 * @author lhx
 * @date 2018年1月4日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_DB_DB_H_
#define CORE_DB_DB_H_

#include <string>
#include <mutex>

#include <thirty-party/SQLiteCpp/include/SQLiteCpp/SQLiteCpp.h>
#include <thirty-party/SQLiteCpp/sqlite3/sqlite3.h>
#include "core/util/time.h"

namespace db {

const std::string DB_PATH = "/sros/db/main.db3";

const int OPEN_FLAGS = SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE | SQLITE_OPEN_NOMUTEX;

const int BUSY_TIMEOUT = 3000;  // ms

}  // namespace db

// 有一个线程可以锁多次 要用的地方加上 std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);
extern std::recursive_mutex g_db_mutex; // 数据库的锁。若同时存在多个锁，数据库的锁锁最里面，以防止死锁

extern SQLite::Database g_db;

#endif  // CORE_DB_DB_H_
