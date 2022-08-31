//
// Created by lfc on 2021/10/12.
//

#ifndef SROS_TCP_SCAN_DATA_RECEIVER_HPP
#define SROS_TCP_SCAN_DATA_RECEIVER_HPP
#include <array>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <condition_variable>

#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"
#include "data_receiver_interface.hpp"
#include "leimou_laser_protocol.hpp"

namespace sros {

class TcpScanDataReceiver : public DataReceiverInterface {
public:
    TcpScanDataReceiver(std::shared_ptr<LaserProtocolInterface> laser_protocol, const std::string hostname,
                        const int tcp_port)
        : inbuf_(4096), instream_(&inbuf_), laser_protocol_(laser_protocol), ring_buffer_(65536), scan_data_() {
        last_data_time_ = std::time(0);
        tcp_socket_ = 0;    //空指针
        is_connected_ = false;
        LOG(INFO) << "Connecting to TCP data channel at " << hostname << ":" << tcp_port << " ... ";
        try {
            // Resolve hostname/ip
            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::asio::ip::tcp::resolver::query query(hostname, std::to_string(tcp_port));    //query查询
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
            boost::asio::ip::tcp::resolver::iterator end;

            tcp_socket_ = new boost::asio::ip::tcp::socket(io_service_);
            boost::system::error_code error = boost::asio::error::host_not_found;
            LOG(INFO) << "will connect!";
            // Iterate over endpoints and etablish connection
            while (error && endpoint_iterator != end) {
                tcp_socket_->close();
                LOG(INFO) << "connect~";
                tcp_socket_->connect(*endpoint_iterator++, error);
                tcp_socket_->set_option(boost::asio::ip::tcp::no_delay(true));
            }
            if (error) throw boost::system::system_error(error);
            LOG(INFO) << "successfully to connect!";
            std::vector<std::vector<char>> start_cmd_data;
            laser_protocol_->getStartCmd(start_cmd_data);

            if (start_cmd_data.size()) {
              LOG(INFO) << "will send cmd data!";
              LOG(INFO) << "data size:" << start_cmd_data.size();
              for (auto &data : start_cmd_data) {
                auto size = tcp_socket_->write_some(boost::asio::buffer(data));
//                usleep(50);
                LOG(INFO) << "size:" << size;
              }


            }
            // Start async reading 开始异步读取
            boost::asio::async_read(
                *tcp_socket_, inbuf_,
                boost::bind(&TcpScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error));
            io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
            is_connected_ = true;
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

    //-----------------------------------------------------------------------------
    void handleSocketRead(const boost::system::error_code& error) {
        if (!error) {
            // Read all received data and write it to the internal ring buffer
            instream_.clear();
            while (!instream_.eof()) {
                char buf[4096];
                instream_.read(buf, 4096);
                int bytes_read = instream_.gcount();
                writeBufferBack(buf, bytes_read);
            }
            // Handle (read and parse) packets stored in the internal ring buffer
            while (handleNextPacket(ring_buffer_)) {
            }

            // Read data asynchronously
            boost::asio::async_read(
                *tcp_socket_, inbuf_,
                boost::bind(&TcpScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error));
        } else {
            if (error.value() != 995)
                std::cerr << "ERROR: "
                          << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
            disconnect();
        }
    }

    bool handleNextPacket(boost::circular_buffer<char>& ring_buffer) {
        // Search for a packet
        int start_id = laser_protocol_->findPackageStart(ring_buffer);
        LOG(INFO) << "start id:" << start_id;
        // if (start_id < 0) {

        //     LOG(INFO) << "start_id < 0";
        //     return false;

        // }
        // Try to retrieve packet
        if (!laser_protocol_->resolvePackage(ring_buffer, start_id)){  // TODO:这里是否需要clearscan数据？
            // LOG(INFO) << "Try to retrieve packet";
            return false;
        }
        // Create new scan container if get a full scan package.
        if (laser_protocol_->isEndPackage()) {
            // LOG(INFO) << "isEndPackage";
            if (laser_protocol_->checkScanValid()) {
                // Lock internal outgoing data queue, automatically unlocks at end of function
                // LOG(INFO) << "checkScanValid";
                std::unique_lock<std::mutex> lock(data_mutex_);
                last_data_time_ = std::time(0);
                scan_data_.emplace_back(new ScanMsg);
                auto& curr_scan = scan_data_.back();
                laser_protocol_->cpToScan(curr_scan);
                while (scan_data_.size() > 2) {
                    scan_data_.pop_front();
                    LOG(WARNING) << "Too many scans in receiver queue: Dropping scans!" << std::endl;
                }
                data_notifier_.notify_one();
            }
        }else{
            // LOG(INFO) << "package is invalue";
        }
        auto& curr_scan = scan_data_.back();
        return true;
    }

    bool getScan(std::shared_ptr<ScanMsg>& scan){
        std::unique_lock<std::mutex> lock(data_mutex_);
        int count = 0;
        while( checkConnection() && isConnected() && scan_data_.empty()) {
            count++;
            if (count > 1) {
                LOG(WARNING) << "the count is:" << count;
            }
            data_notifier_.wait_for(lock, std::chrono::seconds(1));
        }
        if( !scan_data_.empty() && isConnected() )
        {
            scan = (std::move(scan_data_.front()));
            scan_data_.pop_front();
            return true;
        }
        return false;
    }

    //-----------------------------------------------------------------------------
    void disconnect() {
        is_connected_ = false;
        try {
            if (tcp_socket_) tcp_socket_->close();
            io_service_.stop();
            if (boost::this_thread::get_id() != io_service_thread_.get_id()) io_service_thread_.join();
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

    bool isConnected() const { return is_connected_; }

    bool checkConnection()
    {
        if( !isConnected() )
            return false;
        double delta_time = std::time(0) - last_data_time_;
        if( delta_time > 1 ) {
            LOG(WARNING) << "the delta time is large! will return false and reconnect!" << delta_time;
            disconnect();
            LOG(INFO) << "disconnect !";
            return false;
        }
        return true;
    }

    //-----------------------------------------------------------------------------
    void writeBufferBack(char* src, std::size_t numbytes) {
        if (ring_buffer_.size() + numbytes > ring_buffer_.capacity()) throw std::exception();
        ring_buffer_.resize(ring_buffer_.size() + numbytes);
        char* pone = ring_buffer_.array_one().first;
        std::size_t pone_size = ring_buffer_.array_one().second;
        char* ptwo = ring_buffer_.array_two().first;
        std::size_t ptwo_size = ring_buffer_.array_two().second;

        if (ptwo_size >= numbytes) {
            std::memcpy(ptwo + ptwo_size - numbytes, src, numbytes);
        } else {
            std::memcpy(pone + pone_size + ptwo_size - numbytes, src, numbytes - ptwo_size);
            std::memcpy(ptwo, src + numbytes - ptwo_size, ptwo_size);
        }
    }

 private:
    //! Internal connection state
    bool is_connected_;
    //! Receiving socket
    //! Boost::Asio streambuffer
    boost::asio::streambuf inbuf_;

    //! Input stream
    std::istream instream_;

    //! Receiving socket
    boost::asio::ip::tcp::socket* tcp_socket_;

    //! Event handler thread
    boost::thread io_service_thread_;   //thread 线程
    boost::asio::io_service io_service_;

    //! Internal ringbuffer for temporarily storing reveived data
    boost::circular_buffer<char> ring_buffer_;

    //! Protection against data races between ROS and IO threads
    std::mutex data_mutex_;

    //! Data notification condition variable
    std::condition_variable data_notifier_;

    //! Double ended queue with successfully received and parsed data, organized as single complete scans
    std::deque<std::shared_ptr<ScanMsg>> scan_data_; // rename to scans

    std::shared_ptr<LaserProtocolInterface> laser_protocol_;

    //! time in seconds since epoch, when last data was received
    double last_data_time_;
};

}  // namespace sros

#endif  // SROS_TCP_SCAN_DATA_RECEIVER_HPP
