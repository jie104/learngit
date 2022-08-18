//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "server.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <boost/bind.hpp>
#include <glog/logging.h>
#include <boost/algorithm/clamp.hpp>
#include "core/settings.h"

namespace network {
namespace server {

using namespace boost::asio;


server::server(const std::string &address, const std::string &port) :
        io_service_(),
        acceptor_(io_service_, {ip::tcp::v4(), 5001}),
        socket_(io_service_),
        connection_manager_() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_tcp_keep_alive = (s.getValue<std::string>("network.enable_tcp_keep_alive", "False") == "True");
    if (enable_tcp_keep_alive) {
        LOG(INFO) << "tcp keep alive enabled!";

        int keepAlive = 1;  // 开启keepalive属性
        // 如该连接在1800秒内没有任何数据往来,则进行探测
        int keepIdle = s.getValue<int>("network.tcp_keepalive_time", 10);
        int keepInterval = s.getValue<int>("network.tcp_keepalive_intvl", 1);  // 探测时发包的时间间隔为3秒
        // 探测尝试的次数.如果第1次探测包就收到响应了,则后几次的不再发.
        int keepCount = s.getValue<int>("network.tcp_keepalive_probes", 1);
        keepIdle = boost::algorithm::clamp(keepIdle, 10, 7200);
        keepInterval = boost::algorithm::clamp(keepInterval, 1, 75);
        keepCount = boost::algorithm::clamp(keepCount, 1, 30);

        setsockopt(acceptor_.native_handle(), SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
        setsockopt(acceptor_.native_handle(), SOL_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
        setsockopt(acceptor_.native_handle(), SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
        setsockopt(acceptor_.native_handle(), SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));
    }

    do_accept();
}

void server::run() {
    io_service_.run();
}

void server::do_accept() {
    acceptor_.async_accept(socket_,
                           [this](boost::system::error_code ec) {
                               if (!ec) {
                                   auto c = std::make_shared<connection>(std::move(socket_), connection_manager_);

                                   c->set_msg_callback(boost::bind(&server::on_message, this, _1, _2));
                                   c->set_msg_sent_callback(boost::bind(&server::on_message_sent, this, _1, _2));
                                   c->set_disconnected_callback(boost::bind(&server::on_connection_close, this, _1));

                                   c->start();

                                   connect_callback_func_(c->get_peer_ip(), c->get_peer_port());
                               }

                               do_accept();
                           });
}

void server::close(const std::string &ip_addr, unsigned short port) {
    connection_ptr conn = connection_manager_.get_connection(ip_addr, port);
    if (conn) {
        connection_manager_.stop(conn);
    }
}

void server::stop() {
    // The server is stopped by cancelling all outstanding asynchronous
    // operations. Once all operations have finished the io_service::run() call
    // will exit.
    acceptor_.close();
    connection_manager_.stop_all();
}

bool server::send(proto::Message_ptr msg) {
    // 将msg发送到所有connection上
    connection_manager_.foreachConnection([&](connection_ptr conn) {
        conn->write(msg);
    });

    return true;
}

/// send msg to specified connection!
bool server::send(proto::Message_ptr msg, const std::string &ip, unsigned short port) {
    auto conn = connection_manager_.get_connection(ip, port);
    if (conn != nullptr) {
        conn->write(msg);
        return true;
    }

    return false;
}

void server::on_message_sent(proto::Message_ptr msg, connection_ptr conn) {
    std::string peer_ip = conn->get_peer_ip();
    unsigned short peer_port = conn->get_peer_port();
    //std::cout << "send msg to "<<peer_ip <<":"<<peer_port<< " type "<< msg->type()<< ", size "<<msg->ByteSizeLong()<<std::endl;

    if (msg_sent_callback_func_) {
        msg_sent_callback_func_(msg, peer_ip, peer_port);
    }
}

void server::on_message(proto::Message_ptr msg, connection_ptr conn) {
    std::string peer_ip = conn->get_peer_ip();
    unsigned short peer_port = conn->get_peer_port();

    //std::cout << peer_ip <<":"<<peer_port<< " on_message, type "<< msg->type()<< ", size "<<msg->ByteSizeLong()<<std::endl;

    if (msg_callback_func_) {
        msg_callback_func_(msg, peer_ip, peer_port);
    }
}

void server::on_connection_close(connection_ptr conn) {
    LOG(INFO) << "server::on_connection_close()";

    // 连接关闭时触发回调
    std::string peer_ip = conn->get_peer_ip();
    auto port = conn->get_peer_port();

    if (disconnect_callback_func_) {
        disconnect_callback_func_(peer_ip, port);
    }
}

void server::set_msg_sent_callback_func(msg_sent_callback_func_t f) {
    msg_sent_callback_func_ = f;
}

void server::set_msg_callback_func(msg_callback_func_t f) {
    msg_callback_func_ = f;
}

void server::set_connected_callback_func(server::connected_callback_func_t f) {
    connect_callback_func_ = f;
}

void server::set_disconnected_callback_func(server::disconnected_callback_func_t f) {
    disconnect_callback_func_ = f;
}


} // namespace server
} // namespace network

