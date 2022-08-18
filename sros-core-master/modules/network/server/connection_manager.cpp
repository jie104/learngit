//
// connection_manager.cpp
// ~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "connection_manager.h"

#include <algorithm>
#include <iterator>
#include <boost/bind.hpp>

#include <glog/logging.h>

namespace network {
namespace server {

void connection_manager::add(connection_ptr c) {
    std::lock_guard<std::mutex> lock(mutex_);

    connections_.insert(c);
}

connection_ptr connection_manager::get_connection(const std::string &ip, unsigned int port) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it_delete = connections_.end();
    for (auto conn : connections_) {
        if (conn->get_peer_ip() == ip && conn->get_peer_port() == port) { // it->first is key, it->second is value
            return conn;
        }
    }
    return nullptr;
}

void connection_manager::stop(connection_ptr c) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (c == nullptr || connections_.find(c) == connections_.end()) {
        LOG(WARNING) << "connection_manager::stop() connection " << c << " not exist";
        return;
    }

    LOG(INFO) << "connection_manager::stop(): " << c;
    LOG(INFO) << "connection_manager::stop():  connections_.size() = " << connections_.size();

    connections_.erase(c);
    c->stop();

    LOG(INFO) << "connection_manager::stop():  connections_.size() = " << connections_.size();
}

void connection_manager::stop_all() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto &it : connections_) {
        it->stop();
    }
    connections_.clear();
}

void connection_manager::foreachConnection(std::function<void(connection_ptr)> fun) const {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto conn : connections_) {
        fun(conn);
    }
}

} // namespace server
} // namespace network
