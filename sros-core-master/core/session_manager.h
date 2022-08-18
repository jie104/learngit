/**
 * @file session_manager.h
 *
 * @author lhx
 * @date 18-06-25.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_SESSION_MANAGER_H_
#define CORE_SESSION_MANAGER_H_

#include <boost/thread/shared_mutex.hpp>
#include <map>
#include <memory>
#include <vector>
#include <atomic>
#include "core/util/time.h"

namespace sros {
namespace core {

class SessionItem {
 public:
    SessionItem(const std::string &username, const std::string &ip, unsigned short port)
        : username(username), ip_addr(ip), ip_port(port) {
        updateAliveTime();
    }

    virtual ~SessionItem() = default;

    friend std::ostream &operator<<(std::ostream &os, const SessionItem &item) {
        os << "SessionItem: " << item.session_id << "(" << item.username << ", " << item.ip_addr << ", "
           << item.ip_port << ", " << item.is_connected << ")";
        return os;
    }

    void updateAliveTime() { last_alive_time_.store(util::get_time_in_ms()); }
    bool checkIsAlive() {
        auto cur_time = sros::core::util::get_time_in_ms();

        if (cur_time - last_alive_time_ > KEEP_ALIVE_TIME_) {
            return false;
        }
        return true;
    }

    uint64_t session_id = 0;
    std::string username;
    std::string ip_addr;
    uint16_t ip_port;
    bool is_connected = false;

    std::atomic_uint_fast64_t last_alive_time_; // 上一次的活跃时间
    const int KEEP_ALIVE_TIME_ = 5000; // 一个session包活时间
};

using SessionItem_ptr = std::shared_ptr<SessionItem>;

class SessionManager {
 public:
    SessionManager() = default;

    ~SessionManager() = default;

    SessionItem_ptr addItem(const std::string &username, const std::string &ip, unsigned short port);

    SessionItem_ptr addItem(
        SessionItem_ptr item);  // 用户可以通过这个接口定制相关，用户的SessionItem,用户需要指定username, ip, port

    SessionItem_ptr getItem(const std::string &ip, unsigned short port) const;

    SessionItem_ptr getItem(uint64_t session_id) const;

    /**
     * 根据用户名获取session item，主要针对预定义用户，如获fms用户
     * @param user_name
     * @return
     */
    SessionItem_ptr getConnectedItem(const std::string &user_name) const;

    bool removeItem(uint64_t session_id);

    bool removeItem(const SessionItem_ptr &item);

    bool hasItem(uint64_t session_id) const;

    bool empty() const;

    std::vector<SessionItem_ptr> getItemList() const;

    // NOTE: 为什么要标记为连接，而不是直接删除呢？
    // 这样若连接的ip和port相同，就可以认为是同一个客户连接的
    void toggleItemConnected(uint64_t session_id, bool connected);

    void updateItemIPPort(uint64_t session_id, unsigned short port);

    int connectCount() const;

    bool isUserConnected(const std::string &user_name) const;

 private:
    uint64_t generateSessionID();

    mutable boost::shared_mutex mutex_;  // 给map_加锁
    std::map<uint64_t, SessionItem_ptr> map_;
};

}  // namespace core
}  // namespace sros

#endif  // CORE_SESSION_MANAGER_H_
