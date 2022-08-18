/**
 * @file key_value_db.h
 *
 * @author pengjiali
 * @date 19-7-3.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_KEY_VALUE_DB_H_
#define CORE_KEY_VALUE_DB_H_

#include <glog/logging.h>
#include <string>
#include <sstream>
#include "db/db.h"

namespace sros {
namespace core {

class KeyValueDB {
public:
    explicit KeyValueDB(const std::string &table_name)
            : table_name_(table_name), db_(&g_db) {}

    virtual ~KeyValueDB() = default;

protected:
    template<typename T>
    T getValueSlave(const std::string &key, const T &v, bool exist_is_valid_column = true);

    template<typename T>
    bool setValueSlave(const std::string &key, const T &v);

    template<typename T>
    T convertType(const std::string &str, const T &default_value) const;

    template<typename T>
    bool insertValue(const std::string &key, const T &v);

    template<typename T>
    bool updateValue(const std::string &key, const T &v);

    const std::string table_name_;
    SQLite::Database *db_;
};

template<typename T>
T KeyValueDB::getValueSlave(const std::string &key, const T &v, bool exist_is_valid_column) {
    std::string sql;
    std::string rst;

    if (exist_is_valid_column) {
        sql = "SELECT value FROM " + table_name_ + " WHERE KEY = '" + key + "' and is_valid=1;";
    } else {
        sql = "SELECT value FROM " + table_name_ + " WHERE KEY = '" + key + "';";
    }

    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);
        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            rst = query.getColumn("value").getText();
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
    }

    return convertType<T>(rst, v);
}

template<typename T>
bool KeyValueDB::setValueSlave(const std::string &key, const T &v) {
    if (!updateValue(key, v)) {
        return false;
    }
    return true;
}

template<typename T>
T KeyValueDB::convertType(const std::string &str, const T &default_value) const {
    if (str.empty() || str == " " || str == "NA") {
        return default_value;
    }

    std::istringstream iss(str);
    T num;
    iss >> num;
    return num;
}

template<typename T>
bool KeyValueDB::insertValue(const std::string &key, const T &v) {
    std::stringstream sqlstream;
    std::string rst;
    sqlstream << "INSERT INTO " << table_name_ << " (id,key,value) VALUES (NULL,'" << key << "', '" << (v) << "')";
    const std::string &sql = sqlstream.str();
    int r = 0;

    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);
        r = db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
    }

    return r > 0;
}

template<typename T>
bool KeyValueDB::updateValue(const std::string &key, const T &v) {
    std::stringstream sqlstream;
    std::string rst;
    sqlstream << "UPDATE " << table_name_ << " SET value = '" << (v) << "' WHERE key = '" << key << "';";
    const std::string &sql = sqlstream.str();
    int r = 0;

    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);
        r = db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
    }

    return r > 0;
}

}  // namespace core
}  // namespace sros

#endif  // CORE_KEY_VALUE_DB_H_
