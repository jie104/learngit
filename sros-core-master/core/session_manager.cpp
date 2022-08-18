/**
 * @file session_manager.cpp
 *
 * @author lhx
 * @date 18-06-25.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "session_manager.h"
#include <core/util/time.h>
#include <glog/logging.h>

namespace sros {
namespace core {

SessionItem_ptr SessionManager::addItem(const std::string &username, const std::string &ip, unsigned short port) {
    if (getItem(ip, port)) {
        LOG(WARNING) << "session item is exist!";
        return nullptr;
    }

    auto item = std::make_shared<SessionItem>(username, ip, port);
    item->is_connected = true;
    item->session_id = generateSessionID();

    boost::lock_guard<boost::shared_mutex> look(mutex_);
    // 确保同一时刻只存在唯一一个ip和port
    for (auto it = map_.cbegin(); it != map_.cend(); ++it) {
        auto itemm = it->second;
        if (itemm->ip_addr == ip && itemm->ip_port == port) {
            if (itemm->is_connected) {
                LOG(ERROR) << "UNREACHABLE! The same ip and port login twice! ip: " << itemm->ip_addr
                           << " port: " << itemm->ip_port;
                return nullptr;
            } else {
                map_.erase(it);
                break;  // 没加入一个都尝试一次，所以最多存在一次
            }
        }
    }
    map_[item->session_id] = item;

    if (item->session_id == 0) {
        LOG(ERROR) << "item->session_id == 0";
    }

    LOG(INFO) << "add session: " << item->session_id << " ip: " << item->ip_addr << " port: " << item->ip_port;

    return item;
}

SessionItem_ptr SessionManager::addItem(SessionItem_ptr item) {
    if (item) {
        boost::lock_guard<boost::shared_mutex> look(mutex_);
        // 确保同一时刻只存在唯一一个ip和port
        for (auto it = map_.cbegin(); it != map_.cend(); ++it) {
            auto itemm = it->second;
            if (itemm->ip_addr == item->ip_addr && itemm->ip_port == item->ip_port) {
                if (itemm->is_connected) {
                    LOG(ERROR) << "UNREACHABLE! The same ip and port login twice! ip: " << itemm->ip_addr
                               << " port: " << itemm->ip_port;
                    return nullptr;
                } else {
                    map_.erase(it);
                    LOG(INFO) << "The last same ip and port is disconnect with out login, forced to clear!";
                    break;  // 没加入一个都尝试一次，所以最多存在一次
                }
            }
        }

        item->session_id = generateSessionID();
        item->is_connected = true;

        map_[item->session_id] = item;
    } else {
        LOG(ERROR) << "item is nullptr!!!";
    }

    return item;
}

SessionItem_ptr SessionManager::getItem(const std::string &ip, unsigned short port) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    for (const auto& it : map_) {
        auto item = it.second;
        if (item->ip_addr == ip && item->ip_port == port) {
            return item;
        }
    }
    return nullptr;
}

SessionItem_ptr SessionManager::getItem(uint64_t session_id) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    auto it = map_.find(session_id);
    if (it == map_.end()) {
        LOG(INFO) << "Can not find session id: " << session_id;
        return nullptr;
    }

    return it->second;
}

SessionItem_ptr SessionManager::getConnectedItem(const std::string &user_name) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    for (const auto& it : map_) {
        const auto& item = it.second;
        if (item->username == user_name && item->is_connected) {
            return item;
        }
    }
    return nullptr;
}

bool SessionManager::removeItem(uint64_t session_id) {
    boost::lock_guard<boost::shared_mutex> look(mutex_);

    LOG(INFO) << "remove session: " << session_id;
    return map_.erase(session_id) > 0;
}

bool SessionManager::removeItem(const SessionItem_ptr &item) {
    boost::lock_guard<boost::shared_mutex> look(mutex_);

    LOG(INFO) << "remove session: " << item->session_id << " ip: " << item->ip_addr << " port: " << item->ip_port;

    return map_.erase(item->session_id) > 0;
}

uint64_t SessionManager::generateSessionID() { return sros::core::util::get_timestamp_in_ms(); }

bool SessionManager::hasItem(uint64_t session_id) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    auto it = map_.find(session_id);
    return it != map_.end();
}

bool SessionManager::empty() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    return map_.empty();
}

std::vector<SessionItem_ptr> SessionManager::getItemList() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    std::vector<SessionItem_ptr> list;

    for (auto it : map_) {
        list.push_back(it.second);
    }

    return list;
}

void SessionManager::toggleItemConnected(uint64_t session_id, bool connected) {
    boost::lock_guard<boost::shared_mutex> look(mutex_);

    auto it = map_.find(session_id);
    if (it == map_.end()) {
        LOG(INFO) << "Can not find session id: " << session_id;
        return;
    }

    it->second->is_connected = connected;
}

void SessionManager::updateItemIPPort(uint64_t session_id, unsigned short port) {
    boost::lock_guard<boost::shared_mutex> look(mutex_);

    auto it = map_.find(session_id);
    if (it == map_.end()) {
        LOG(INFO) << "Can not find session id: " << session_id;
        return;
    }

    it->second->ip_port = port;
}

int SessionManager::connectCount() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    return map_.size();
}
bool SessionManager::isUserConnected(const std::string &user_name) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    bool ret = false;
    for (auto it : map_) {
        auto item = it.second;
        if (item->username == user_name && item->is_connected) {
            ret = true;
        }
    }
    return ret;
}

}  // namespace core
}  // namespace sros
