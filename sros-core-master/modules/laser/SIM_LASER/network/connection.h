//
// Created by lhx on 16-1-21.
//

#ifndef SRC_SDK_DEMO_CONNECTION_H
#define SRC_SDK_DEMO_CONNECTION_H

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <memory>

#include "connect_msg/connect_msg_factory.hpp"

namespace network {
namespace client{


/**
 * 处理与服务器的连接以及发送、接受数据包
 */
class Connection {
public:
    typedef boost::function<void(connection::BaseMsg_ptr)> msg_callback_func_t;
    typedef boost::function<void(void)> disconnect_callback_func_t;

    Connection(bool enable_udp);
    virtual ~Connection();

    bool connect(const char* server_ip, int server_port);

    void disconnect();

    bool sendMsg(connection::BaseMsg_ptr msg);

    bool sendUDPMsg(connection::BaseMsg_ptr msg);

    // AbstractMsg
    template <typename T>
    std::shared_ptr<T> receiveMsg(); // receive a msg from network, block function

    bool isNetworkOkay();

    void setMsgCallback(msg_callback_func_t callback_f);

    void setDisconnectCallback(disconnect_callback_func_t f);

private:
    typedef boost::asio::ip::tcp::endpoint endpoint_t;
    typedef boost::asio::ip::tcp::socket socket_t;
    typedef boost::asio::ip::address address_t;
    typedef boost::shared_ptr<boost::thread> thread_ptr;
    typedef boost::asio::ip::udp::socket udp_socket_t;

    void close();

    connection::BaseMsg_ptr _read_msg();
    void _handle_msg(connection::BaseMsg_ptr read_msg);

    bool _write_some_bytes(unsigned char* data, int length);
    bool _read_some_bytes(unsigned char* data, int length);

    void _start_recv_thread();
    void _recv_thread();

    boost::asio::io_service io_service_;
    boost::system::error_code ec_;
    socket_t socket_;

    bool enable_udp_; // 是否启用udp

    udp_socket_t udp_socket_;

    boost::asio::ip::udp::endpoint udp_endpoint_;

    msg_callback_func_t callback_f_;
    disconnect_callback_func_t disconnect_callback_f_;

    connection::BaseMsg header_msg_;

    thread_ptr recv_thread_;
    thread_ptr udp_recv_thread_;

    bool recv_thread_running_;
    bool udp_recv_pose_thread_running_;

};

}

} /* namespace network */

#endif //SRC_SDK_DEMO_CONNECTION_H
