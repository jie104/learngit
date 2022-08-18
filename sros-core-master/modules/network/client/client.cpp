//
// client.hpp
// ~~~~~~~~~~~~~~~
// written by Penglei 
//
//

#include <iostream>
#include <boost/bind.hpp>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "core/module_manager.h"
#include "client.h"

namespace network {
namespace client {

using namespace boost::asio;

//client::client(boost::asio::io_service& io_service, boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
client::client(std::string recver_ip, std::string recver_port)
    :recver_ip_(recver_ip),
    recver_port_(recver_port),
    io_service_(),
    socket_(io_service_)
{
    //nothing
}
void client::setMessageCallbackFunc(MessageCallbackFunc_t callback) {
    msg_callback_f_ =  callback;
}
void client::start(){
    //receiving frame from endpoint!
    boost::asio::async_read(socket_,
                            boost::asio::buffer(recv_frame_.data, proto::Frame::FRAME_HEADER_LEN),
                            boost::bind(&client::handle_read_header,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred)
                            );
}
void client::handle_read_header(const boost::system::error_code& e, std::size_t bytes_transferred) {
    
    if (!e) {
        LOG(ERROR) << "handle read header";
        size_t frame_size = proto::Frame::getFrameBodySize(recv_frame_);
        if (!frame_size) {
            close();
            return;
        }
        recv_frame_.total_size = proto::Frame::FRAME_HEADER_LEN + frame_size;

        boost::asio::async_read(socket_,
                                boost::asio::buffer(recv_frame_.get_frame_data(), frame_size),
                                boost::bind(&client::handle_read_body, 
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
    } else if (e != boost::asio::error::operation_aborted) {
        LOG(ERROR) << "error handle read header";
        close();
    }
}
void client::handle_read_body(const boost::system::error_code& e, std::size_t bytes_transferred) {
    if (!e) {
        LOG(ERROR) << "handle read body";
        proto::Message_ptr msg = proto::Frame::buildMessage(recv_frame_);
        
        msg_callback_f_(msg);

        boost::asio::async_read(socket_,
                                boost::asio::buffer(recv_frame_.data, proto::Frame::FRAME_HEADER_LEN),
                                boost::bind(&client::handle_read_header, 
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
    } else if (e != boost::asio::error::operation_aborted){
        LOG(ERROR) << "error handle read body";
        close();
    }
}

bool client::send(const proto::Message_ptr msg) {
    try {
        //do_connect();
        proto::Frame_ptr frame = proto::Frame::buildFrame(msg);
        startTransmit(frame);
        return true;
    
    } catch (boost::system::system_error &e) {
        std::cout << "Error in Recver: " << e.what() << std::endl;
        return false;
    }

}
void client::connect()
{
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    endpoint_t ep(boost::asio::ip::address::from_string(recver_ip_.c_str()),
                  (unsigned short)std::stod(recver_port_));
    try {
        socket_.connect(ep);
    } catch (boost::system::system_error &e) {
        std::cout << "Error in Client: " << e.what() << std::endl;
        return;
    }
    io_service_.run();
    //start();
}
void client::close(){
    socket_.close();
}

void client::startTransmit(const proto::Frame_ptr frame){
    if (frame->total_size != 0) {
        socket_.send(boost::asio::buffer(frame->data, (size_t) frame->total_size));
    }
}

}
}
