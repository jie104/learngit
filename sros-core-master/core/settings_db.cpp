/**
 * @file settings_db
 *
 * @author pengjiali
 * @date 20-4-16.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#include "settings_db.h"
#include "util/time.h"
#include "util/timer.h"

namespace sros {
namespace core {

SettingsDB::SettingsDB() : KeyValueDB(SETTINGS_TABLE) {}

SettingsDB::~SettingsDB() {}

bool SettingsDB::setTmpValue(const std::string &key, const std::string &v) {
    std::lock_guard<std::mutex> lg(mutex_);

    tmp_settings_[key] = v;

    return true;
}

bool SettingsDB::setTmpValues(std::map<std::string, std::string> &&values) {
    std::lock_guard<std::mutex> lg(mutex_);

    tmp_settings_ = std::move(values);
    return true;
}

ItemInfo SettingsDB::getItemInfo(const std::string &key) {
    ItemInfo item_info = {};

    std::vector<std::string> keys = {key};
    auto list = getItemInfoLists(keys);

    if (list.size() > 0) {
        item_info = list[0];
    }

    return item_info;
}

bool SettingsDB::insertItem(const ItemInfo &item_info) {
    // 修改的时间戳
    auto changed_time = std::to_string(sros::core::util::get_timestamp_in_s());

    std::string sql_part1 = "INSERT INTO " + SETTINGS_TABLE + " VALUES ";
    std::string sql_part2 = std::string("(") + "NULL" + ",'" + item_info.key + "','" + item_info.name + "','" +
                            item_info.value + "','" + item_info.default_value + "','" + item_info.value_units + "','" +
                            item_info.value_type + "','" + item_info.value_range + "','" + item_info.description +
                            "'," + std::to_string(item_info.permission) + ",'" + changed_time + "','" +
                            item_info.changed_user + "',1)";
    std::string sql = sql_part1 + sql_part2;
    LOG(INFO) << "SQL : " << sql;

    int r = 0;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        r = db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    return r > 0;
}

bool SettingsDB::updateItem(const ItemInfo &item_info) {
    //    Timer t("Settings::updateItem(const ItemInfo &item_info)");

    // 修改的时间戳
    auto changed_time = std::to_string(sros::core::util::get_timestamp_in_s());

    std::string sql_part1 = "UPDATE " + SETTINGS_TABLE + " SET";
    std::string sql_part2 = "  value='" + item_info.value + "',changed_time='" + changed_time + "',changed_user='" +
                            item_info.changed_user + "',value_range='" + item_info.value_range + "' WHERE key = '" +
                            item_info.key + "'";
    std::string sql = sql_part1 + sql_part2;
    LOG(INFO) << "SQL : " << sql;

    int r = 0;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        r = db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }

    return r > 0;
}

bool SettingsDB::setItemInfo(const ItemInfo &item_info) {
    if (!updateItem(item_info)) {
        return false;
    }

    std::lock_guard<std::mutex> lg(mutex_);
    cache_[item_info.key] = item_info.value;
    return true;
}

bool SettingsDB::setItemInfoList(const ItemInfoLists &item_info_list) {
    if (item_info_list.empty()) {
        return false;
    }

    // 数据库批量修改时启用事务能够显著提高执行速度
    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Transaction transaction(*db_);
        for (int i = 0; i < item_info_list.size(); i++) {
            auto rst = setItemInfo(item_info_list[i]);
            if (!rst) {
                return false;
            }
        }
        transaction.commit();
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
        return false;
    }
    return true;
}

ItemInfoLists SettingsDB::getItemInfoLists() {
    std::vector<std::string> keys;

    return getItemInfoLists(keys);
}

ItemInfoLists SettingsDB::getItemInfoLists(const std::vector<std::string> &keys) {
    std::string sql = "SELECT * FROM " + SETTINGS_TABLE + " ";
    std::string condition = "WHERE is_valid=1";

    if (keys.size() == 1) {
        condition = "WHERE key = '" + keys[0] + "' AND is_valid=1";
    } else if (keys.size() > 1) {
        LOG(ERROR) << "SettingsDB::getItemInfoLists(): keys.size() >1";
        return ItemInfoLists();
    }

    sql = sql + condition + ";";

    return getItemInfoList(sql);
}

ItemInfoLists SettingsDB::getItemInfoListOfClass(const std::string &class_name) {
    std::string sql = "SELECT * FROM " + SETTINGS_TABLE + " WHERE key LIKE '" + class_name + ".%' AND is_valid=1;";
    return getItemInfoList(sql);
}

ItemInfoLists SettingsDB::getItemInfoList(const std::string &sql) {
    LOG(INFO) << "SQL : " << sql;

    ItemInfoLists item_info_lists;
    std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
    std::lock_guard<std::mutex> lg(mutex_);

    try {
        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            ItemInfo item_info;
            item_info.id = query.getColumn(0).getInt();
            item_info.key = query.getColumn(1).getString();
            item_info.name = query.getColumn(2).getString();
            item_info.value = query.getColumn(3).getString();
            item_info.default_value = query.getColumn(4).getString();
            item_info.value_units = query.getColumn(5).getString();
            item_info.value_type = query.getColumn(6).getString();
            item_info.value_range = query.getColumn(7).getString();
            item_info.description = query.getColumn(8).getString();
            item_info.permission = query.getColumn(9).getInt();
            item_info.changed_time = query.getColumn(10).getString();
            item_info.changed_user = query.getColumn(11).getString();

            // 获取对于temporary，若缓存中有数据就用缓存中的数据，若没有就用数据库中的数据
            if (item_info.key.find("temporary") == 0) {
                auto iter = cache_.find(item_info.key);
                if (iter != cache_.end()) {
                    item_info.value = cache_[item_info.key];
                } else {
                    cache_.insert(std::make_pair(item_info.key, item_info.value));
                }
            }
            item_info_lists.push_back(item_info);
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }
    return item_info_lists;
}

std::string SettingsDB::getSectionName(const std::string &key) {
    char delimiter = '.';

    if (key.find(delimiter) == -1) {
        return std::string();
    }

    std::stringstream ss(key);
    std::string item;

    while (std::getline(ss, item, delimiter)) {
        return item;
    }

    return std::string();
}
ItemInfo SettingsDB::findItemByID(int32_t id, const ItemInfoLists &list) {
    for (auto item : list) {
        if (item.id == id) {
            return item;
        }
    }
    return ItemInfo();
}

}  // namespace core
}  // namespace sros