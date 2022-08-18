//
// Created by lhx on 16-2-26.
//

#ifndef SROS_USER_MANAGER_H
#define SROS_USER_MANAGER_H

#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <thirty-party/SQLiteCpp/include/SQLiteCpp/Database.h>
#include <thirty-party/SQLiteCpp/include/SQLiteCpp/Statement.h>

#include "core/state.h"

namespace sros {
namespace core {
/**
 * CREATE TABLE login(
    id INTEGER PRIMARY KEY,
    session_id BIGINT NOT NULL,
    username TEXT,
    passwd TEXT,
    permission TEXT,
    login_time BIGINT,
    item_valid INT
);
 */

const int PERMISSION_USER = 10; // user权限，数字版
const int PERMISSION_ADMIN = 20; // admin权限，数字版
const int PERMISSION_DEPLOY = 30; // deploy权限，数字版
const int PERMISSION_DEV = 40; // dev权限，数字版
const int PERMISSION_ROOT = 50; // root权限，数字版

const std::string USERNAME_FMS = "fms"; // 调度系统该用户名
const std::string USERNAME_SROS = "sros"; // 调度系统该用户名

typedef struct {
    std::uint32_t ID;
    std::uint64_t session_id;
    std::string username;
    std::string passwd;
    std::string permission;
    std::uint64_t login_time;
    int item_valid;
} UserItem;

typedef std::vector<UserItem> UserItemList;

typedef std::map<std::string, UserItem> UserItemCache_t;

class UserManager {
public:
    static UserManager &getInstance();

    UserItem getUserItem(const std::string &username);

    bool setUserItem(UserItem item);

    UserItemList getUserItemList();

    static std::string getPermissionStr(int permission);
private:
    UserManager();

    UserManager(UserManager const &);

    UserManager &operator=(UserManager const &);

    ~UserManager();

    bool insertUserItem(UserItem item);

    bool updateUserItem(UserItem login_item_info);

    UserItemCache_t cache_;

    SQLite::Database *db_;

    const std::string TABLE_NAME = "login";
};

}
}


#endif //SROS_USER_MANAGER_H
