//
// Created by lfc on 2021/10/12.
//

#ifndef SROS_UDP_SCAN_DATA_RECEIVER_HPP
#define SROS_UDP_SCAN_DATA_RECEIVER_HPP
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
namespace sros {

class UdpScanDataReceiver :public DataReceiverInterface{
 public:
    UdpScanDataReceiver(std::shared_ptr<LaserProtocolInterface> laser_protocol, const std::string hostname,
                        const int udp_port)
        : inbuf_(4096),
          instream_(&inbuf_),
          laser_protocol_(laser_protocol),
          ring_buffer_(65536),
          udp_port_(udp_port),
          hostname_(hostname),
          scan_data_() {
        last_data_time_ = std::time(0);
        udp_socket_ = 0;
        is_connected_ = false;

        try {
            boost::asio::ip::udp::endpoint send_point(boost::asio::ip::address::from_string(hostname), udp_port);
            udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
            udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));//设置udp_port，那连接上了以后，该数据通过该port回传
            LOG(INFO) << "Connecting to UDP data channel at " << hostname << ":" << udp_port << " rcv port: " << udp_socket_->local_endpoint().port();
            std::vector<std::vector<char>> start_cmd_data;
            laser_protocol_->getStartCmd(start_cmd_data);

            if (start_cmd_data.size()) {
                for (auto& cmd_data : start_cmd_data) {
                    LOG(INFO) << "will send cmd data!";
                    auto size = udp_socket_->send_to(boost::asio::buffer(cmd_data), send_point);
                    usleep(50);
                }
            }
            udp_socket_->async_receive_from(
                boost::asio::buffer(&udp_buffer_[0], udp_buffer_.size()), udp_endpoint_,
                boost::bind(&UdpScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
            io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
            is_connected_ = true;

            LOG(INFO) << "successfully to connect!";
        } catch (std::exception& e) {
            disconnect();
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

    void sendCmd(const std::vector<char>& data) {
        udp_socket_->send_to(boost::asio::buffer(data.data(), data.size()), udp_endpoint_);
    }

    //-----------------------------------------------------------------------------
    void handleSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if (!error) {
            //            LOG(INFO) << "handle no error!";
            // Read all received data and write it to the internal ring buffer
            writeBufferBack(&udp_buffer_[0], bytes_transferred);

            // Handle (read and parse) packets stored in the internal ring buffer
            while (handleNextPacket(ring_buffer_)) {
            }

            // Read data asynchronously
            udp_socket_->async_receive_from(
                boost::asio::buffer(&udp_buffer_[0], udp_buffer_.size()), udp_endpoint_,
                boost::bind(&UdpScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        } else {
            if (error.value() != 995)
                std::cerr << "ERROR: "
                          << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
            disconnect();
        }
    }

    bool handleNextPacket(boost::circular_buffer<char>& ring_buffer) {
        // Search for a packet
        int packet_start = laser_protocol_->findPackageStart(ring_buffer);
        if (packet_start < 0) return false;
        // Try to retrieve packet
        if (!laser_protocol_->resolvePackage(ring_buffer, packet_start))  // TODO:这里是否需要clearscan数据？
        {
            LOG(INFO) << "cant resolvePackage the message " << hostname_;
            return false;
        }
        // Create new scan container if necessary
        if (laser_protocol_->isEndPackage()) {
            if (laser_protocol_->checkScanValid()) {
                // Lock internal outgoing data queue, automatically unlocks at end of function
                std::unique_lock<std::mutex> lock(data_mutex_);
                last_data_time_ = std::time(0);
                scan_data_.emplace_back(new ScanMsg);
                auto& curr_scan = scan_data_.back();
                laser_protocol_->cpToScan(curr_scan);
                while (scan_data_.size() > 100) {
                    scan_data_.pop_front();
                    LOG(WARNING) << "Too many scans in receiver queue: Dropping scans!" << std::endl;
                }
                data_notifier_.notify_one();
            } else {
                LOG(INFO) << "!laser_protocol_->checkScanValid() " << hostname_;
            }
        }
        return true;
    }

    bool getScan(std::shared_ptr<ScanMsg>& scan) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        int count = 0;
        while (checkConnection() && isConnected() && scan_data_.empty()) {
            count++;
            if (count > 1) {
                LOG(WARNING) << "the count is:" << count <<" " << hostname_;
            }
            data_notifier_.wait_for(lock, std::chrono::seconds(1));
        }
        if (!scan_data_.empty() && isConnected()) {
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
            if (udp_socket_) udp_socket_->close();
            io_service_.stop();
            if (boost::this_thread::get_id() != io_service_thread_.get_id()) io_service_thread_.join();
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

    bool isConnected() const { return is_connected_; }

    bool checkConnection() {
        if (!isConnected()) return false;
        double delta_time = std::time(0) - last_data_time_;
        if (delta_time > 1) {
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
    std::string hostname_;
    //! Internal connection state
    bool is_connected_;
    //! Receiving socket
    //! Boost::Asio streambuffer
    boost::asio::streambuf inbuf_;

    //! Input stream
    std::istream instream_;

    //! Receiving socket
    boost::asio::ip::udp::socket* udp_socket_;

    //! Event handler thread
    boost::thread io_service_thread_;
    boost::asio::io_service io_service_;

    //! Internal ringbuffer for temporarily storing reveived data
    boost::circular_buffer<char> ring_buffer_;

    //! Protection against data races between ROS and IO threads
    std::mutex data_mutex_;

    //! Data notification condition variable
    std::condition_variable data_notifier_;

    //! Double ended queue with sucessfully received and parsed data, organized as single complete scans
    std::deque<std::shared_ptr<ScanMsg>> scan_data_;

    std::shared_ptr<LaserProtocolInterface> laser_protocol_;

    //! time in seconds since epoch, when last data was received
    double last_data_time_;

    //! Endpoint in case of UDP receiver
    boost::asio::ip::udp::endpoint udp_endpoint_;

    //! Buffer in case of UDP receiver
    std::array<char, 65536> udp_buffer_;
    int udp_port_;
};

}  // namespace sros

#endif  // SROS_TCP_SCAN_DATA_RECEIVER_HPP
