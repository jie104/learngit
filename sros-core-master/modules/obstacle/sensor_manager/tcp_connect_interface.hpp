//
// Created by lfc on 2020/9/2.
//

#ifndef SROS_TCP_CONNECT_INTERFACE_HPP
#define SROS_TCP_CONNECT_INTERFACE_HPP
#include <array>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace sensor{
typedef boost::function<void(char* ,int)> decodeDataFunc;

class TcpConnectInterface {
 public:
    TcpConnectInterface(const decodeDataFunc &decodeDataCallback,const std::string &host_name, const int tcp_port)
        : inbuf_(4096), instream_(&inbuf_), decodeData(decodeDataCallback), is_connected_(false) {
        try
        {
            // Resolve hostname/ip
            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::asio::ip::tcp::resolver::query query(host_name, std::to_string(tcp_port));
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
            boost::asio::ip::tcp::resolver::iterator end;

            tcp_socket_ = new boost::asio::ip::tcp::socket(io_service_);
            boost::system::error_code error = boost::asio::error::host_not_found;

            // Iterate over endpoints and etablish connection
            while (error && endpoint_iterator != end)
            {
                tcp_socket_->close();
                tcp_socket_->connect(*endpoint_iterator++, error);
            }
            if (error)
                throw boost::system::system_error(error);

            // Start async reading
            boost::asio::async_read(*tcp_socket_, inbuf_, boost::bind(&TcpConnectInterface::handleSocketRead, this, boost::asio::placeholders::error));
            io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
            is_connected_ = true;
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " <<  e.what() << std::endl;
        }
    }

    void handleSocketRead(const boost::system::error_code& error)
    {
        if (!error )
        {
            // Read all received data and write it to the internal ring buffer
            instream_.clear();
            while(!instream_.eof())
            {
                char buf[4096];
                instream_.read(buf,4096);
                int bytes_read = instream_.gcount();
                if (decodeData) {
                    decodeData(buf,bytes_read);
                }
            }

            // Read data asynchronously
            boost::asio::async_read(*tcp_socket_, inbuf_, boost::bind(&TcpConnectInterface::handleSocketRead, this, boost::asio::placeholders::error));
        }
        else
        {
            if( error.value() != 995 )
                std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
            disconnect();
        }
    }

    bool isConnected() const { return is_connected_; }

    void disconnect()
    {
        is_connected_ = false;
        try
        {
            if( tcp_socket_ )
                tcp_socket_->close();
            io_service_.stop();
            if( boost::this_thread::get_id() != io_service_thread_.get_id() )
                io_service_thread_.join();
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " <<  e.what() << std::endl;
        }
    }

 private:

    boost::thread io_service_thread_;
    boost::asio::io_service io_service_;

    //! Boost::Asio streambuffer
    boost::asio::streambuf inbuf_;

    //! Input stream
    std::istream instream_;

    //! Receiving socket
    boost::asio::ip::tcp::socket* tcp_socket_;

    decodeDataFunc decodeData;

    bool is_connected_ = false;

};

}

#endif  // SROS_TCP_CONNECT_INTERFACE_HPP
