#include "ord/ord_driver_net.h"
#include "ord/utils.h"
#include "ord/lidar_address.h"
#include <asio.hpp>
#include <thread>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string.h>

namespace ord_sdk
{

class Network : public Net
{
public:
  Network(const LidarAddress& sensor);

  virtual void close();
  virtual bool isOpened() const;

  virtual void TransmitSubmit(std::vector<uint8_t> message);
  virtual error_t open();

private:
  void ReceiveHandler(const asio::error_code& error, size_t bytes_transferred);
  void TransmitHandler(const asio::error_code& error, size_t);

private:
  std::thread worker_thread_;

  asio::io_service io_service_;

  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint device_address_;
  asio::ip::udp::endpoint sender_address_;
  std::deque<std::vector<uint8_t>> transmit_message_queue_;

  std::array<uint8_t, PACKET_LENGTH_MAX> receive_packet_buffer_;
};

Network::Network(const LidarAddress& sensor)
  : device_address_(asio::ip::address_v4(ntohl(sensor.address())), ntohs(sensor.port()))
  , socket_(io_service_)
{

}

void Network::close()
{
  if (socket_.is_open()) {
    socket_.close();
    worker_thread_.join();
  }
}

error_t Network::open()
{
  socket_.open(asio::ip::udp::v4());

  asio::error_code bind_result;
  asio::ip::udp::endpoint local_ep(asio::ip::address_v4(),0);

  socket_.bind(local_ep, bind_result);
  if (!bind_result) {
    std::cout << "local_ep is bind!" << std::endl;
  }else{
    return address_in_use;
  }
  socket_.async_receive_from(asio::buffer(receive_packet_buffer_),
                                sender_address_,
                                std::bind(&Network::ReceiveHandler,
                                          this, std::placeholders::_1, std::placeholders::_2));

  worker_thread_ = std::thread([&]() {
    io_service_.run();
  });                                        
#if 1
  std::cout << "local_ep.address is " << local_ep.address().to_string() << std::endl;
  std::cout << "local_ep.port is " << socket_.local_endpoint().port() << std::endl;

  std::cout << "sensor_address_.address is " << device_address_.address().to_string() << std::endl;
  std::cout << "sensor_address_.port is " << device_address_.port()<< std::endl;
#endif
  return error_t::no_error;
}

bool Network::isOpened() const
{
  return socket_.is_open();
}

void Network::TransmitSubmit(std::vector<uint8_t> message)
{
  std::shared_ptr<std::vector<uint8_t>> wrapped_message = std::make_shared<std::vector<uint8_t>>(std::move(message));
  io_service_.dispatch([this, wrapped_message]() {
    bool transmit_in_progress = !transmit_message_queue_.empty(); 
    transmit_message_queue_.push_back(std::move(*wrapped_message));
    if (!transmit_in_progress) {
      std::vector<uint8_t>& send_data = transmit_message_queue_.front();      
      socket_.async_send_to(asio::buffer(send_data),
                                    device_address_,
                                    std::bind(&Network::TransmitHandler,
                                              this, std::placeholders::_1, std::placeholders::_2));
    }
  });
}

void Network::ReceiveHandler(const asio::error_code& error, size_t bytes_transferred)
{
  uint32_t CRCResult = 0;
  if (!error) {
    if (received_message_callback_) { 
      //if (ntohs(*((uint16_t *)receive_packet_buffer_.data())) == bytes_transferred) 
      {
        if (*(receive_packet_buffer_.data()+2) == 0x00)
        {
          CRCResult = Utils::calcCrc32(receive_packet_buffer_.data(),bytes_transferred-4);
        }

        if(CRCResult == ntohl(*(uint32_t*)(receive_packet_buffer_.data()+bytes_transferred-4)) || 
        ((*(receive_packet_buffer_.data()+0) == 0x4D) && (*(receive_packet_buffer_.data()+1) == 0x53) && (*(receive_packet_buffer_.data()+2) == 0x01) && \
        (*(receive_packet_buffer_.data()+3) == 0xF4) && (*(receive_packet_buffer_.data()+4) == 0xEB) && (*(receive_packet_buffer_.data()+5) == 0x90) && \
        ntohs(*((uint16_t *)(receive_packet_buffer_.data()+6))) == bytes_transferred)){
          std::vector<uint8_t> recv_data(receive_packet_buffer_.data(),
                                receive_packet_buffer_.data() + bytes_transferred);
          received_message_callback_(std::move(recv_data));
        }
      }
    }
    socket_.async_receive_from(asio::buffer(receive_packet_buffer_),
                                   sender_address_,
                                   std::bind(&Network::ReceiveHandler,
                                             this, std::placeholders::_1, std::placeholders::_2));
  }
}

void Network::TransmitHandler(const asio::error_code& error, size_t)
{
    transmit_message_queue_.pop_front();

    if (!transmit_message_queue_.empty()){ 
      std::vector<uint8_t>& transmit_message = transmit_message_queue_.front();
      socket_.async_send_to(asio::buffer(transmit_message),
                                    device_address_,
                                    std::bind(&Network::TransmitHandler,
                                              this, std::placeholders::_1, std::placeholders::_2));
    }
}


std::unique_ptr<Net> Net::create(const SocketAddress& sensor)
{
  if (typeid(sensor) == typeid(LidarAddress))
    return std::unique_ptr<Network>(
      new Network(dynamic_cast<const LidarAddress&>(sensor)));
  else
    return nullptr;
}


void Net::setReceivedMessageCallback(Net::ReceivedMessageCallback callback)
{
  received_message_callback_ = callback;
}


}
