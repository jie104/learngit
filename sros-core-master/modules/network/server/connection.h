//
// connection.hpp
// ~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef HTTP_CONNECTION_HPP
#define HTTP_CONNECTION_HPP

#include <deque>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>

#include "core/proto/frame.h"

namespace network {
namespace server {

class connection;
class connection_manager;

typedef std::shared_ptr<connection> connection_ptr;

/// Represents a single connection from a client.
class connection:
        public std::enable_shared_from_this<connection>,
        private boost::noncopyable {
public:
    /// Construct a connection with the given io_service.
    explicit connection(boost::asio::ip::tcp::socket socket,
                        connection_manager& manager);

    ~connection();

    /// Get the socket associated with the connection.
    boost::asio::ip::tcp::socket& socket();

    /// Start the first asynchronous operation for the connection.
    bool start();

    /// Stop all asynchronous operations associated with the connection.
    void stop();

    void write(const proto::Message_ptr& msg);

    typedef boost::function<void(proto::Message_ptr, connection_ptr)> msg_sent_callback_func_t;
    void set_msg_sent_callback(msg_sent_callback_func_t callback);
    
    typedef boost::function<void(proto::Message_ptr, connection_ptr)> msg_callback_func_t;
    void set_msg_callback(msg_callback_func_t callback);

    typedef boost::function<void(connection_ptr)> connected_callback_func_t;
    void set_connected_callback(connected_callback_func_t callback);

    typedef connected_callback_func_t disconnected_callback_func_t;
    void set_disconnected_callback(disconnected_callback_func_t callback);

    std::string get_peer_ip();
    unsigned short get_peer_port();
private:

    void do_read_header();

    void do_read_body();

    void do_write();

    void handle_write(const proto::Frame_ptr& frame);

    /// Socket for the connection.
    boost::asio::ip::tcp::socket socket_;

    /// The manager for this connection.
    connection_manager& connection_manager_;

    /// Buffer for incoming data.
    proto::Frame recv_frame_;

    typedef std::deque<proto::Frame_ptr> FrameQueue;

    /// Buffer for outgoing data.
    FrameQueue frame_queue_;

    std::string peer_ip_;
    unsigned short peer_port_;

    bool is_okay_ = false; // connection是否有效

    msg_sent_callback_func_t msg_sent_callback_f_;
    msg_callback_func_t msg_callback_f_;
    connected_callback_func_t connected_callback_f_;
    disconnected_callback_func_t disconnected_callback_f_;
};

//typedef connection* connection_ptr;

} // namespace server
} // namespace network

#endif // HTTP_CONNECTION_HPP
