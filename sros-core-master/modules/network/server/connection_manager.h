//
// connection_manager.hpp
// ~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef HTTP_CONNECTION_MANAGER_HPP
#define HTTP_CONNECTION_MANAGER_HPP

#include <set>
#include <boost/noncopyable.hpp>
#include <mutex>

#include "connection.h"

namespace network {
namespace server {

/// Manages open connections so that they may be cleanly stopped when the server
/// needs to shut down.
class connection_manager: private boost::noncopyable {
public:
    /// Add the specified connection to the manager and start it.
    void add(connection_ptr c);

    /// Stop the specified connection.
    void stop(connection_ptr c);

    /// Stop all connections.
    void stop_all();

    connection_ptr get_connection(const std::string &ip, unsigned int port) const;

    typedef std::set<connection_ptr> connections_t;

    void foreachConnection(std::function<void (connection_ptr)> fun) const;

private:
    /// The managed connections.
    connections_t connections_;
    mutable std::mutex mutex_; // 防止多个线程同时操作connections_t
};

} // namespace server
} // namespace network

#endif // HTTP_CONNECTION_MANAGER_HPP
