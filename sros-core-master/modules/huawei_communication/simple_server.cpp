//
// Created by john on 18-9-28.
//

#include "simple_server.h"
#include <cstdlib>
#include <deque>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <utility>
#include <boost/asio.hpp>
#include <glog/logging.h>

#include "../../core/msg/huawei_comm_data_msg.hpp"

using boost::asio::ip::tcp;

namespace huawei {

SimpleServer::SimpleServer(short port)
        : acceptor_(io_service_, tcp::endpoint(tcp::v4(), port)), socket_(io_service_) {
    doAccept();
}

void SimpleServer::doAccept() {
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
        if (!ec) {
            LOG(INFO) << "HuaweiCommModule: new connection";
            std::make_shared<SimpleSession>(std::move(socket_), msg_callback_func_)->start();
//            if (session_ptr_ && session_ptr_->isOpen()) {
//                // 当已经有链接了并且在链接中，后来的链接不处理
////            } else {
//                session_ptr_ = std::make_shared<SimpleSession>(std::move(socket_), msg_callback_func_);
//                session_ptr_->start();
//            }
        }

        doAccept();
    });
}

SimpleSession::~SimpleSession() {
    LOG(INFO) << "~SimpleServer()";
}
void SimpleSession::doReadHeader() {
    auto self(shared_from_this());
    msg_.raw_data_.resize(msg_.header_length_);
    boost::asio::async_read(socket_,
                            boost::asio::buffer(msg_.raw_data_.data(), msg_.header_length_),
                            [this, self](boost::system::error_code ec, std::size_t length) {
//                                LOG(INFO) << length;
                                if (!ec) {
                                    msg_.init();
                                    doReadBody(msg_.getLeftLength());
                                } else {
                                    LOG(INFO) << ec.message();
                                    LOG(INFO) << "SimpleSession error 1-------------------------------";
                                }
                            });
}

void SimpleSession::doReadBody(int length) {
    auto self(shared_from_this());
    msg_.raw_data_.resize(msg_.header_length_ + length);
    boost::asio::async_read(socket_,
                            boost::asio::buffer(msg_.raw_data_.data() + msg_.header_length_, length),
                            [this, self](boost::system::error_code ec, std::size_t length) {
                                if (!ec) {
                                    std::stringstream ss;
                                    for (auto it = msg_.raw_data_.cbegin(); it != msg_.raw_data_.cend(); ++it) {
                                        ss << std::hex << (int) *it << " ";
                                    }
//                                    LOG(INFO) << ss.str();

                                    if (!msg_.decodeRawData()) { // 数据不正确直接退出
                                        return;
                                    }

                                    if (msg_callback_func_ != 0) {
                                        sros_ready_ = false;
                                        msg_callback_func_(shared_from_this());
                                        // 阻塞等待
                                        std::unique_lock<std::mutex> lk(mutex_);
                                        condistion_variable_.wait(lk, [&]{return sros_ready_;});
                                    }
                                    doWrite();
                                } else {
                                    LOG(INFO) << "SimpleSession error";
                                }
                            });
}

void SimpleSession::itIsTimeToResponse() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        sros_ready_ = true;
    }
    condistion_variable_.notify_one();
}

void SimpleSession::doWrite() {
    auto self(shared_from_this());
    msg_.generateRawData();
    boost::asio::async_write(socket_,
                             boost::asio::buffer(msg_.raw_data_.data(),
                                                 msg_.raw_data_.size()),
                             [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                                 if (!ec) {
                                     doReadHeader();
                                 }
                             });
}

}