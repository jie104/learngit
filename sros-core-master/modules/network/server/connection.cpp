//
// connection.cpp
// ~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "connection.h"

#include <boost/bind.hpp>

#include "core/module_manager.h"    // LOG() function
#include "connection_manager.h"

namespace network {
namespace server {

using boost::asio::ip::tcp;

connection::connection(tcp::socket socket, connection_manager& manager) :
        socket_(std::move(socket)),
        connection_manager_(manager),
        is_okay_(false) {
}

connection::~connection() {
    LOG(INFO) << this << "~connection()";
}

boost::asio::ip::tcp::socket& connection::socket() {
    return socket_;
}

void connection::set_msg_callback(msg_callback_func_t callback) {
    msg_callback_f_ = callback;
}

void connection::set_msg_sent_callback(msg_sent_callback_func_t callback) {
    msg_sent_callback_f_ = callback;
}

bool connection::start() {
    try {
        peer_ip_ = socket_.remote_endpoint().address().to_string();
        peer_port_ = socket_.remote_endpoint().port();
    } catch (boost::system::system_error& e) {
        LOG(ERROR) << "fail to get remote endpoint :" << e.what();
        // connection_manager_.stop(shared_from_this());
        return false;
    }

    boost::system::error_code ec;
    socket_.set_option(boost::asio::ip::tcp::no_delay(true), ec);
    if (ec) {
        LOG(INFO) << "set TCP_NODELAY " << ec.message();
    }

    connection_manager_.add(shared_from_this());

    if (connected_callback_f_) {
        connected_callback_f_(shared_from_this());
    }

    LOG(INFO) << this << " connection start";

    do_read_header();

    is_okay_ = true;

    return true;
}

void connection::do_read_header() {
    using namespace boost::asio;
    auto self(shared_from_this());

    async_read(socket_,
               buffer(recv_frame_.data, proto::Frame::FRAME_HEADER_LEN),
               [this, self](boost::system::error_code ec, std::size_t length) {
                   if (!ec && proto::Frame::getFrameBodySize(recv_frame_) > 0) {
                       do_read_body();
                   } else {
                       LOG(ERROR) << this << " do_read_header(): error_no " << ec.value() << ":" << ec.message();
                       connection_manager_.stop(shared_from_this());
                   }
               });
}

void connection::do_read_body() {
    using namespace boost::asio;
    auto self(shared_from_this());

    size_t frame_size = proto::Frame::getFrameBodySize(recv_frame_);
    recv_frame_.total_size = proto::Frame::FRAME_HEADER_LEN + frame_size;

    async_read(socket_,
               buffer(recv_frame_.get_frame_data(), frame_size),
               [this, self](boost::system::error_code ec, std::size_t length) {
                   if (!ec) {
                       proto::Message_ptr msg = proto::Frame::buildMessage(recv_frame_);
                       msg_callback_f_(msg, shared_from_this());

                       do_read_header();
                   } else {
                       LOG(ERROR) << this << " do_read_body(): error_no " << ec.value() << ":" << ec.message();
                       connection_manager_.stop(shared_from_this());
                   }
               });
}


void connection::write(const proto::Message_ptr& msg) {
    if (!is_okay_) {
        LOG(WARNING) << this << " connection::write(): connection is not okay";
        return;
    }

    int MAX_ALLOW_FRAME_QUEUE_SIZE = 100;
    if (frame_queue_.size() > MAX_ALLOW_FRAME_QUEUE_SIZE) {
        LOG(WARNING) << this << " connection::write(): frame queue > " << MAX_ALLOW_FRAME_QUEUE_SIZE
            << ", terminate connection";
        connection_manager_.stop(shared_from_this());
        return;
    }

    proto::Frame_ptr frame = proto::Frame::buildFrame(msg);

    // 防止并发问题，将入队列操作放在io_service事件循环中处理
    socket_.get_io_service().post(boost::bind(&connection::handle_write, shared_from_this(), frame));
}

void connection::handle_write(const proto::Frame_ptr& frame) {
    bool write_in_progress = !frame_queue_.empty();  // 是否正在发送

    if (frame_queue_.size() > 1) {
        LOG_IF_EVERY_N(WARNING, frame_queue_.size() > 20, 10) << this << " handle_write(): frame_queue size is "
            << frame_queue_.size();
    }

    frame_queue_.push_back(frame);

    if (!write_in_progress) {
        do_write();
    }
}

void connection::do_write() {
    using namespace boost::asio;
    auto self(shared_from_this());

    async_write(socket_,
                buffer(frame_queue_.front()->data,
                       frame_queue_.front()->total_size),
                [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                    if (!ec) {
                        frame_queue_.pop_front();

                        if (frame_queue_.size() > 1) {
                            LOG_IF_EVERY_N(WARNING, frame_queue_.size() > 20, 10) << this
                                << " do_write(): frame_queue size is " << frame_queue_.size();
                        }

                        if (!frame_queue_.empty()) {
                            do_write();
                        }
                    } else {
                        LOG(ERROR) << this << " do_write(): error_no " << ec.value() << ":" << ec.message();
                        connection_manager_.stop(shared_from_this());
                    }
                });
}


void connection::stop() {
    boost::system::error_code ec;

    if (!is_okay_) {
        LOG(WARNING) << this << " connection has been stopped";
        return;
    }
    is_okay_ = false;

    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket_.close();

    if (ec) {
        LOG(ERROR) << this << " connection::stop(): Exception: " << ec.message();
    }

    if (disconnected_callback_f_) {
        LOG(INFO) << this << " connection::stop(): trigger disconnected_callback";
        disconnected_callback_f_(shared_from_this());
    }
}

std::string connection::get_peer_ip() {
    return peer_ip_;
}

uint16_t connection::get_peer_port() {
    return peer_port_;
}

void connection::set_connected_callback(connection::connected_callback_func_t callback) {
    connected_callback_f_ = callback;
}

void connection::set_disconnected_callback(connection::disconnected_callback_func_t callback) {
    disconnected_callback_f_ = callback;
}

}  // namespace server
}  // namespace network
