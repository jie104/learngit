//
// Created by lhx on 16-2-26.
//

#include "user_manager.h"

#include <glog/logging.h>
#include <thirty-party/SQLiteCpp/include/SQLiteCpp/SQLiteCpp.h>
#include <thirty-party/SQLiteCpp/sqlite3/sqlite3.h>

#include "core/db/db.h"

namespace sros {
namespace core {

UserManager::UserManager() : db_(&g_db) {
    std::cout << SQLite::getLibVersion() << std::endl;
}

UserManager::~UserManager() {}

UserManager::UserManager(const UserManager &login_manager) : db_(&g_db) {}

UserManager &UserManager::operator=(const UserManager &login_manager) { return *this;}

UserManager &UserManager::getInstance() {
    static UserManager login_manager;
    return login_manager;
}

UserItem UserManager::getUserItem(const std::string &username) {
    UserItem item;

    auto iter = cache_.find(username);
    if (iter != cache_.end()) {
        return cache_[username];
    }

    std::string sql = "SELECT * FROM " + TABLE_NAME + " WHERE username = \"" + username + "\";";
    std::cout << "get sql " << sql << std::endl;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            item.ID = query.getColumn(0).getInt();
            item.session_id = query.getColumn(1).getUInt64();
            item.username = query.getColumn(2).getString();
            item.passwd = query.getColumn(3).getString();
            item.permission = query.getColumn(4).getString();
            item.login_time = query.getColumn(5).getUInt64();
            item.item_valid = query.getColumn(6).getInt();
        }

        cache_[item.username] = item;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }
    return item;
}

bool UserManager::insertUserItem(UserItem item) {
    bool rst = true;
    std::string sql_part1 = "INSERT INTO " + TABLE_NAME + " VALUES ";
    std::string sql_part2 = std::string("(") + "NULL" + "," + std::to_string(item.session_id) + ",'" + item.username +
                            "','" + item.passwd + "','" + item.permission + "'," + std::to_string(item.login_time) +
                            "," + std::to_string(item.item_valid) + ")";
    std::string sql = sql_part1 + sql_part2;
    std::cout << "insert sql string : " << sql << std::endl;
    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        rst = db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }
    return rst;
}

bool UserManager::updateUserItem(UserItem login_item_info) {
    bool rst = true;

    std::string sql_part1 = "UPDATE " + TABLE_NAME + " SET";
    std::string sql_part2 = " session_id=" + std::to_string(login_item_info.session_id) + ",username='" +
                            login_item_info.username + "',passwd='" + login_item_info.passwd + "',permission='" +
                            login_item_info.permission + "',login_time=" + std::to_string(login_item_info.login_time) +
                            ",item_valid=" + std::to_string(login_item_info.item_valid) + " WHERE username = \"" +
                            login_item_info.username + "\" ;";
    std::string sql = sql_part1 + sql_part2;
    std::cout << "update sql string : " << sql << std::endl;

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        rst = db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }
    // std::cout<<"rst=============="<<rst<<std::endl;
    return rst;
}

bool UserManager::setUserItem(UserItem item) {
    bool rst = updateUserItem(item);
    if (!rst) {
        rst = insertUserItem(item);
    }

    return rst;
}

UserItemList UserManager::getUserItemList() {
    UserItemList item_list;
    std::string sql = "SELECT * FROM " + TABLE_NAME + ";";

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            UserItem item;
            item.ID = query.getColumn(0).getInt();
            item.session_id = query.getColumn(1).getUInt64();
            item.username = query.getColumn(2).getString();
            item.passwd = query.getColumn(3).getString();
            item.permission = query.getColumn(4).getString();
            item.login_time = query.getColumn(5).getUInt64();
            item.item_valid = query.getColumn(6).getInt();
            cache_.insert(std::make_pair(item.username, item));
            item_list.push_back(item);
        }

    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what();
    }
    return item_list;
}

std::string UserManager::getPermissionStr(int permission) {
    std::string str;
    switch (permission) {
        case PERMISSION_USER: {
            str = "user";
            break;
        }
        case PERMISSION_ADMIN: {
            str = "admin";
            break;
        }
        case PERMISSION_DEPLOY: {
            str = "deploy";
            break;
        }
        case PERMISSION_DEV: {
            str = "dev";
            break;
        }
        case PERMISSION_ROOT: {
            str = "root";
            break;
        }
        default: {
            LOG(ERROR) << "UNREAVHABLE! permission is " << permission;
            str = "unkown";
            break;
        }
    }
    return str;
}

}  // namespace core
}  // namespace sros
