//
// Created by john on 18-9-28.
//

#ifndef SROS_SIMPLE_SERVER_H
#define SROS_SIMPLE_SERVER_H

#include <array>
#include <boost/asio.hpp>
#include <memory>
#include <boost/function.hpp>
#include <mutex>
#include <condition_variable>
#include "comm_msg.h"


namespace huawei {
class SimpleSession;
using SimpleSession_Ptr = std::shared_ptr<SimpleSession>;
using msg_callback_func_t = boost::function<void(SimpleSession_Ptr session_ptr)>;

class SimpleSession : public std::enable_shared_from_this<SimpleSession> {
public:
    SimpleSession(boost::asio::ip::tcp::socket socket, msg_callback_func_t f) : socket_(std::move(socket)),
    msg_callback_func_(f) {}
    ~SimpleSession();

    void start() { doReadHeader(); }
    bool isOpen() { return socket_.is_open(); }

    CommMsg &getMsg() { return msg_; } // 外界会修改这里面的内容
    void itIsTimeToResponse(); // 本函数有外部线程来调用

private:
    void doReadHeader();

    void doReadBody(int length);

    void doWrite();

    int id_; // 标示，用于同时处理多个链接的情况

    msg_callback_func_t msg_callback_func_ = 0;
    boost::asio::ip::tcp::socket socket_;
    CommMsg msg_;

    std::mutex mutex_;
    std::condition_variable condistion_variable_;
    bool sros_ready_ = false; // 记录sros是否将需要回复的消息处理好
};


class SimpleServer {
public:
    SimpleServer(short port);

    void run() { io_service_.run(); }

    void set_msg_callback_func(msg_callback_func_t f) { msg_callback_func_ = f; }

private:
    void doAccept();

    std::shared_ptr<SimpleSession> session_ptr_;

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    msg_callback_func_t msg_callback_func_ = 0;
};

}


#endif //SROS_SIMPLE_SERVER_H
