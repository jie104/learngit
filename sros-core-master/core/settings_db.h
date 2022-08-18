/**
 * @file settings_db
 *
 * @author pengjiali
 * @date 20-4-16.
 *
 * @describe 这一层只有简单的数据库
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_SETTINGS_DB_H
#define SROS_SETTINGS_DB_H

#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>

#include "key_value_db.h"

 #define TRIGMODE_AUTO					0x90
 #define TRIGMODE_FPGA					0x91
 #define TRIGMODE_SOFT					0x92
 #define TRIGMODE_SIGNAL				0x93

namespace sros {
namespace core {

const static std::string SETTINGS_TABLE = "config";

typedef std::map<std::string, std::string> SettingsCache;

class ItemInfo {
 public:
    std::int32_t id;
    std::string key;
    std::string name;
    std::string value;
    std::string default_value;
    std::string value_units;
    std::string value_type;
    std::string value_range;
    std::string description;
    std::int32_t permission;
    std::string changed_time;
    std::string changed_user;
};

typedef std::vector<ItemInfo> ItemInfoLists;

class SettingsDB : public KeyValueDB {
 public:
    template <typename T>
    T getValue(const std::string &key, const T &v);  // 首先获取临时参数，若没有临时参数，就获取正常的参数

    template <typename T>
    bool setValue(const std::string &key, const T &v);  // 设置正常参数

    bool setTmpValue(const std::string &key, const std::string &v);  // 设置临时参数
    bool setTmpValues(std::map<std::string, std::string> &&values);  // 替换所有临时参数

    ItemInfo getItemInfo(const std::string &key);

    bool setItemInfo(const ItemInfo &item_info);

    ItemInfoLists getItemInfoLists();

    ItemInfoLists getItemInfoLists(const std::vector<std::string> &keys);

    /**
     * @brief 获取某一类的所有配置
     * @param class_name 类名，如获取slam 所有配置，输入slam
     * @return 某一类的所有配置
     */
    ItemInfoLists getItemInfoListOfClass(const std::string &class_name);

    bool setItemInfoList(const ItemInfoLists &item_info_list);

    // 从key中提取section（以'.'做分隔）
    static std::string getSectionName(const std::string &key);

    static ItemInfo findItemByID(int32_t id, const ItemInfoLists &list);

 protected:
    SettingsDB();

    virtual ~SettingsDB();

    ItemInfoLists getItemInfoList(const std::string &sql);

    bool insertItem(const ItemInfo &item_info);

    virtual bool updateItem(const ItemInfo &item_info);

 private:
    std::mutex mutex_; // 这个锁同时保护cache_ 和 tmp_settings_
    SettingsCache cache_;  // 正常参数的缓存
    SettingsCache tmp_settings_;  // 临时的设置，当断电重启后就会消失，临时参数的优先级比正常参数的优先级高
};

template <typename T>
T SettingsDB::getValue(const std::string &key, const T &v) {
    {
        std::lock_guard<std::mutex> lg(mutex_);
        auto search = tmp_settings_.find(key);
        if (search != tmp_settings_.end()) {
            return convertType(search->second, v);
        }

        auto iter = cache_.find(key);
        if (iter != cache_.end()) {
            std::string value = cache_[key];
            return convertType<T>(value, v);
        }
    }
    LOG(INFO) << "settings cache missed: " << key;

    T ret = getValueSlave(key, v);

    // 如果没有从数据库里查询到key，仍然会将key插入到cache中，插入的value内容为空字符串
    // 在convertType()最终返回结果时，会将空字符串替换为默认值v
    // 这样可以保证即使数据库中没有key，也不会频繁执行数据库查询影响性能
    std::lock_guard<std::mutex> lg(mutex_);
    cache_.insert(std::make_pair(key, std::to_string(ret)));

    return ret;
}

template <>  // std::string 模板特例
inline std::string SettingsDB::getValue(const std::string &key, const std::string &v) {
    {
        std::lock_guard<std::mutex> lg(mutex_);
        auto search = tmp_settings_.find(key);
        if (search != tmp_settings_.end()) {
            if (search->second.empty() || search->second == " " || search->second == "NA") {
                return v;
            } else {
                return search->second;
            }
        }

        auto iter = cache_.find(key);
        if (iter != cache_.end()) {
            if (iter->second.empty() || iter->second == " " || iter->second == "NA") {
                return v;
            } else {
                return iter->second;
            }
        }
    }
    LOG(INFO) << "settings cache missed: " << key;

    std::string ret = getValueSlave(key, v);

    // 如果没有从数据库里查询到key，仍然会将key插入到cache中，插入的value内容为空字符串
    // 在convertType()最终返回结果时，会将空字符串替换为默认值v
    // 这样可以保证即使数据库中没有key，也不会频繁执行数据库查询影响性能
    std::lock_guard<std::mutex> lg(mutex_);
    cache_.insert(std::make_pair(key, ret));

    return ret;
}

template <typename T>
bool SettingsDB::setValue(const std::string &key, const T &v) {
    if (setValueSlave(key, v)) {
        std::lock_guard<std::mutex> lg(mutex_);
        cache_.erase(key);  // 更新成功，从cache中清除，下次读取时从数据中读取
        return true;
    }

    return false;
}

}  // namespace core
}  // namespace sros

#endif  // SROS_SETTINGS_DB_H
