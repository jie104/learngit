//
// Created by lfc on 2020/11/23.
//

#ifndef SERIAL_READ_PGV_SERIAL_READ_HPP
#define SERIAL_READ_PGV_SERIAL_READ_HPP
#include <boost/asio.hpp>
#include <memory>
#include <glog/logging.h>
#include <boost/function.hpp>
#include <boost/circular_buffer.hpp>
namespace pgv{
typedef boost::function<void(const std::vector<unsigned char>)> SerialCallbackFunc;
typedef boost::function<void(void)> DisconnectCallbackFunc;

class PgvSerialRead {
public:
  PgvSerialRead(const int type) : read_type_(type)
  {
    LOG(INFO)<<"Asesing read type:"<<read_type_;
  }

  bool open(const std::string serial_port,int baud_rate){
    is_open_ = false;
    try {
      LOG(INFO) << "begin to read:" << serial_port << "," << baud_rate;
      serial_port_ = serial_port;
      baud_rate_ = baud_rate;
      bool opened = false;
      int i = 0;
      while(!opened)
       try {
        serial_.reset(new boost::asio::serial_port(io_service_));
        serial_->open(serial_port_);
        opened = true;
      }
      catch(const std::exception& e) {
        ++i;
        usleep(10000);
        if(i <= 20)
          continue;
        else
        {
          LOG(WARNING)<<"Asensing IMU is not  equipment,please check it!!!!!";
          return false;
        }
          
      }

      serial_->set_option(boost::asio::serial_port::baud_rate(baud_rate));
      serial_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
      if (serial_->is_open()) {
        std::vector<uint8_t> write_option;
        if(read_type_ == 0)
        {
          write_option.push_back(0xBD);
          write_option.push_back(0xDB);
          write_option.push_back(0x52);
          write_option.push_back(0x20);
          serial_->write_some(boost::asio::buffer(write_option));
        }
        else
        {
          write_option.push_back(0xBD);
          write_option.push_back(0xDB);
          write_option.push_back(0x54);
          write_option.push_back(0x00);
          serial_->write_some(boost::asio::buffer(write_option));
        }
        write_option.clear();
        write_option.push_back(0xBD);
        write_option.push_back(0xDB);
        write_option.push_back(0x04);
        serial_->write_some(boost::asio::buffer(write_option));

        usleep(10000);

        LOG(INFO) << "successfully to read!";
        is_open_ = true;
        return true;
      }else{
        LOG(INFO) << "cannot open serial:" << serial_port;
      }
      return false;
    } catch (...) {
      LOG(INFO) << "cannot open the serial! please make sure the cmd right!";
      return false;
    }
  }

  void setDataReceiveCallback(const SerialCallbackFunc& callback){
    dataCallback_ = callback;
  }

  void close(){
    is_open_ = false;
  }

  void readLoop(){
    while (is_open_) {
      try{
        int data_size = 54;
        if(read_type_ == 2)
           data_size = 58; // 有yaw_inc的imu使用
        std::vector<unsigned char> datas(data_size,0);
        if(readData(datas,data_size)){
          if (dataCallback_) {
            datas.resize(data_size);
            dataCallback_(datas);
          }
        }else{
          throw;
//          LOG(INFO) << "err to read anything!";
        }
      }catch(std::exception &e){
        LOG(INFO) << "serial response an exception!" << e.what();
        if (disconnectCallback_) {
          disconnectCallback_();
        }

        usleep(10000);
        LOG(INFO) <<  "IMU is in abnormal state\n try reconnect imu...";
        serial_->close();
      try {
        serial_.reset(new boost::asio::serial_port(io_service_));
        serial_->open(serial_port_);
      }
      catch(const std::exception& e) {
        usleep(10000);
        continue;
      }
      
        
        serial_->set_option(boost::asio::serial_port::baud_rate(baud_rate_));
        serial_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        if (serial_->is_open())
        {
          std::vector<uint8_t> write_option;
          if (read_type_ == 0)
          {
            write_option.push_back(0xBD);
            write_option.push_back(0xDB);
            write_option.push_back(0x52);
            write_option.push_back(0x20);
            serial_->write_some(boost::asio::buffer(write_option));
          }
          else
          {
            write_option.push_back(0xBD);
            write_option.push_back(0xDB);
            write_option.push_back(0x54);
            write_option.push_back(0x00);
            serial_->write_some(boost::asio::buffer(write_option));
          }
          write_option.clear();
          write_option.push_back(0xBD);
          write_option.push_back(0xDB);
          write_option.push_back(0x04);
          serial_->write_some(boost::asio::buffer(write_option));
        }
    }
  }
  }

  bool readData(std::vector<unsigned char> &data_read,int& data_size){
    int  read_size = serial_->read_some(boost::asio::buffer(data_read, data_size));
    data_size = read_size;
    return data_size != 0;
  }

  bool xorCheck(const std::vector<uint8_t> &data, uint8_t xor_byte,int length) {
    uint8_t check_xor = 0;
    for (int i = 0; i < length; ++i) {
      check_xor ^= data[i];
    }
    if (check_xor == xor_byte) {
      return true;
    }
    LOG(INFO) << "the length is:" << length << "," << (int)check_xor << ","
              << (int)xor_byte;
    LOG(INFO) << "the xor check is wrong!";
    return false;
  }

  bool resolveData(std::vector<uint8_t>& data,int &x,int16_t &y,int16_t & yaw){
    if (data[1] == 0x45) {
      const int x_direction_max = 0x7fffff,x_offset = 0xffffff;
      const int16_t y_dirction_max = 0x2000, y_offset = 0x4000;
      data[2] = data[2] & 0x07;
      x = ((int)data[2] * 0x80 * 0x4000) + ((int)data[3] * 0x4000) +
          ((int)data[4] * 0x80) + (int)data[5];
      x = x > x_direction_max ? x - x_offset : x;
      y = ((int)data[6] * 0x80) + data[7];
      y = y > y_dirction_max ? y - y_offset : y;
      yaw = ((int)data[10] * 0x80) + data[11];
      return true;
    }
    return false;
  }
private:
  SerialCallbackFunc dataCallback_;
  DisconnectCallbackFunc disconnectCallback_;

  std::shared_ptr<boost::asio::serial_port> serial_;
  boost::asio::io_service io_service_;
  bool is_open_ = false;
  std::string serial_port_;
  int baud_rate_;
  int read_type_;
};


}

#endif // SERIAL_READ_PGV_SERIAL_READ_HPP
