//
// server.hpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef HTTP_SERVER_HPP
#define HTTP_SERVER_HPP

#include <string>

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>

#include "connection.h"
#include "connection_manager.h"

namespace network {
namespace server {

/// The top-level class of the TCP server.
class server: private boost::noncopyable {
public:
    /// Construct the server to listen on the specified TCP address and port
    explicit server(const std::string& address, const std::string& port);

    /// Run the server's io_service loop.
    void run();

    bool send(proto::Message_ptr msg);
    bool send(proto::Message_ptr msg, const std::string &ip, unsigned short port);

    void on_message_sent(proto::Message_ptr msg, connection_ptr conn);
    void on_message(proto::Message_ptr msg, connection_ptr conn);

    void on_connection_close(connection_ptr conn);

    /// close specified connection for application layer
    void close(const std::string &ip_addr, unsigned short port);

    /// Handle a request to stop the server.
    void stop();

    typedef boost::function<void(proto::Message_ptr msg, std::string&, unsigned short)> msg_callback_func_t;
    void set_msg_callback_func(msg_callback_func_t f);

    typedef boost::function<void(proto::Message_ptr msg, std::string&, unsigned short)> msg_sent_callback_func_t;
    void set_msg_sent_callback_func(msg_sent_callback_func_t f);

    typedef boost::function<void(std::string, unsigned short port)> connected_callback_func_t;
    void set_connected_callback_func(connected_callback_func_t f);

    typedef boost::function<void(std::string, unsigned short port)> disconnected_callback_func_t;
    void set_disconnected_callback_func(disconnected_callback_func_t f);
private:
    void do_accept();

    /// The io_service used to perform asynchronous operations.
    boost::asio::io_service io_service_;

    boost::asio::ip::tcp::socket socket_;

    /// Acceptor used to listen for incoming connections.
    boost::asio::ip::tcp::acceptor acceptor_;

    /// The connection manager which owns all live connections.
    connection_manager connection_manager_;

    msg_sent_callback_func_t msg_sent_callback_func_;
    msg_callback_func_t msg_callback_func_;

    connected_callback_func_t connect_callback_func_;
    disconnected_callback_func_t disconnect_callback_func_;

};

} // namespace server
} // namespace network

#endif // HTTP_SERVER_HPP
