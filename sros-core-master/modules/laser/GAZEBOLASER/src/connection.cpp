//
// Created by lhx on 16-1-21.
//

#include "connection.h"

namespace network {

using namespace boost::asio::ip;

#define UDP_SOCKET_PORT 5098

Connection::Connection(bool enable_udp)
        : io_service_(),
          socket_(io_service_),
          udp_socket_(io_service_, udp::endpoint(udp::v4(), UDP_SOCKET_PORT)),
          udp_endpoint_(udp::v4(), UDP_SOCKET_PORT),
          recv_thread_running_(false),
          enable_udp_(enable_udp) {
}

Connection::~Connection() {
    close();
}

bool Connection::connect(const char* server_ip, int server_port) {

    endpoint_t endpoint(address_t::from_string(server_ip), (unsigned short)server_port);
    udp_endpoint_ = udp::endpoint(address_t::from_string(server_ip), UDP_SOCKET_PORT);

    try {
        socket_.connect(endpoint); // 连接到服务器
        // 设置no_delay选项，保证实时性
        socket_.set_option(boost::asio::ip::tcp::no_delay(true));

        _start_recv_thread(); // 启动接受msg线程
    } catch (boost::system::system_error &e) {
        ec_ = e.code();
        printf("connect host failed: (%d)%s\n", e.code().value(),
               e.code().message().c_str());
        return false;
    }
    return true;
}

void Connection::disconnect() {
    close();
    // 调用连接断开的回调
    if (disconnect_callback_f_) {
        disconnect_callback_f_();
    }
}

/**
 * 启动接受线程
 */
void Connection::_start_recv_thread() {
    recv_thread_running_ = true;
    recv_thread_.reset(new boost::thread(boost::bind(&Connection::_recv_thread, this)));
}

/**
 * TODO 使用thread.interrupt()停止线程执行
 */
void Connection::_recv_thread() {
    BaseMsg_ptr msg;
//    printf("start recv thread\n");
    while (!ec_ && recv_thread_running_) {
//        printf("recv thread waiting\n");
        msg = _read_msg();
//        printf("read_a_msg()");
        _handle_msg(msg);
    }
}

/**
 * 向Socket写入一个Msg
 * 阻塞方法
 */
bool Connection::sendMsg(BaseMsg_ptr msg) {
    if (ec_)
        return false;

    msg->encode();
    bool r = _write_some_bytes(msg->data(), msg->getLength());

    if (enable_udp_) {
        // 如果启用udp传输, tcp每次可以发送数据包添加添加2ms延时, 防止出现tcp粘包
        boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    }

    return r;
}


// 使用udp发送pose数据
bool Connection::sendUDPMsg(BaseMsg_ptr msg) {
    if (!enable_udp_) return false;

    msg->encode();
    return udp_socket_.send_to(boost::asio::buffer(msg->data(), msg->getLength()),
                               udp_endpoint_) > 0;
}

bool Connection::_write_some_bytes(unsigned char* data, int length) {
    try {
        boost::asio::write(socket_, boost::asio::buffer(data, length));
    } catch (boost::system::system_error &e) {
        ec_ = e.code();
        printf("_write_some_bytes() failed: (%d)%s\n", e.code().value(),
               e.code().message().c_str());
        return false;
    }
    return true;
}

bool Connection::_read_some_bytes(unsigned char* data, int length) {
    try {
        boost::asio::read(socket_, boost::asio::buffer(data, length));
    } catch (boost::system::system_error &e) {
        ec_ = e.code();
        printf("_read_some_bytes() failed: (%d)%s\n", e.code().value(),
               e.code().message().c_str());
        return false;
    }
    return true;
}

/**
 * 从TCP socket读取一个完整的Msg，调用者处理完Msg后需要对delete Msg
 * 阻塞方法
 * 在_recv_thread()中被调用
 * @return SR::AbstractMsg类型的指针
 */
BaseMsg_ptr Connection::_read_msg() {
    if (ec_)
        return NULL;

    if (!_read_some_bytes((unsigned char *) header_msg_.data(),
                          header_msg_.getHeaderLength())) {
        return NULL;
    }
    header_msg_.decodeHeader();

    BaseMsg_ptr msg = MsgFactory::getMsg(header_msg_.getType());
    msg->setBodyLength(header_msg_.getBodyLength());
    if (!_read_some_bytes((unsigned char *) msg->bodyData(), msg->getBodyLength())) {
        return NULL;
    }

    msg->decode();
    return msg;
}

/**
 * 调用回调函数处理收到的msg
 * 回调函数在_recv_thread()中被调用
 */
void Connection::_handle_msg(BaseMsg_ptr read_msg) {
//    std::cout << "Handle_msg" << std::endl;
    if (!read_msg || !callback_f_)
        return;

    callback_f_(read_msg); // 处理msg的回调函数
}

void Connection::close() {
    recv_thread_running_ = false;
    socket_.close();
    udp_socket_.close();
}

bool Connection::isNetworkOkay() {
    return !ec_;
}

template<typename T>
std::shared_ptr<T> Connection::receiveMsg() {
    auto msg = _read_msg();
    return std::dynamic_pointer_cast<T>(msg);
}

void Connection::setMsgCallback(Connection::msg_callback_func_t callback_f) {
    callback_f_ = callback_f;
}

void Connection::setDisconnectCallback(disconnect_callback_func_t f) {
    disconnect_callback_f_ = f;
}

} /* namespace network */

