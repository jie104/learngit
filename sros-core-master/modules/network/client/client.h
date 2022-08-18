//
// client.hpp
// ~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef CLIENT_HPP_
#define CLIENT_HPP_

#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include <boost/function.hpp>

using boost::asio::ip::tcp;
#include "core/proto/frame.h"

namespace network {
namespace client {

class client
{
public:

    //client(boost::asio::io_service& io_service, boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    client(std::string recver_ip, std::string recver_port);

    void close();
    
    bool send(const proto::Message_ptr msg);
    
    void connect();
    
    void startTransmit(const proto::Frame_ptr frame);

    void start();
    void handle_read_header(const boost::system::error_code& e, std::size_t bytes_transferred);
    void handle_read_body(const boost::system::error_code& e, std::size_t bytes_transferred);
    
    typedef boost::function<void(proto::Message_ptr)> MessageCallbackFunc_t;
    void setMessageCallbackFunc(MessageCallbackFunc_t callback);

private:
    std::string recver_ip_;
    std::string recver_port_;
    //boost::asio::io_service& io_service_;
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;
    proto::Frame recv_frame_;
    MessageCallbackFunc_t msg_callback_f_;

};
typedef std::shared_ptr<client> client_ptr;
typedef network::client::client client_type;
typedef boost::asio::ip::tcp::endpoint endpoint_t;
}//namespace client
}//namespace network

#endif
