/*********************************************************************
*
* Software License Agreement ()
*
*  Copyright (c) 2020, HINS, Inc.
*  All rights reserved.
*
* Author: Hu Liankui
* Create Date: 8/9/2020
*********************************************************************/

/**
 * @brief 兴颂激光雷达数据获取
 */

#pragma once

#define BOOST_CB_DISABLE_DEBUG
#include <string>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
#include <thread>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include "connection_address.h"
#include "hins/protoc.h"

namespace hins{

class LaserDataReceiver;
using LaserDataReceiverPtr = LaserDataReceiver*;
using LaserDataReceiverHdr = std::shared_ptr<LaserDataReceiver>;

class LaserDataReceiver final
{
public:
  LaserDataReceiver(const ConnectionAddress &conn_info);

  ~LaserDataReceiver();

  int SyncWrite();
  int SyncWrite(const std::array<unsigned char, 12> command);
  int SyncWrite(const std::array<char, 18> command);
  bool Connect();
  std::array<unsigned char, 12> GenerateParamCommand(XingSongLaserParam param);
  bool IsConnected();

  void Disconnect();

  bool CheckConnection();

  short int GetLaserSteadyTime();

  ScanData GetFullScan();

  int GetBlockValue()
  {
    return have_block_;
  }

  int GetResponsChannel()
  {
    return _now_channel_;
  }
private:
  void HandleSocketRead(const boost::system::error_code& error);

  int16_t FindPacketStart();

  /**
   * @brief WriteBufferBack write data to buffer tail
   * @param src
   * @param num_bytes
   */
  void WriteBufferBack(char* src, std::size_t num_bytes);

  /**
   * @brief HandleNextPacket parse the data in ring buffer
   * @return true - parse success; false - otherwise
   */
  bool HandleNextPacket();

  bool RetrivePacket();

  void ReadBufferFront(char* dst, const uint16_t& num_bytes);
  bool GetRangeData(int16_t head_index);
  bool GetAreaData(int16_t head_index);

private:
  ConnectionAddress conn_info_;
  bool is_connected_;

  bool area_data_flag_;
  int have_block_;
  int _now_channel_;

  boost::thread io_service_thread_;
  boost::asio::io_service io_service_;
  int scan_all_point_num_;
  double angle_increment_;
  // short int rec_begin_flag_;
  bool scan_all_point_num_init_flag;
  short int laser_steady_time;                    // 等待雷达转速均匀的次数（接收一个完整帧为一次）
  boost::asio::streambuf inbuf_;                  // Boost::Asio streambuffer
  std::istream instream_;                         // Input stream

  boost::asio::ip::tcp::socket* tcp_socket_ptr_;  // reveiving socket

  boost::circular_buffer<char> ring_buffer_;      // Internal ringbuffer for temporarily storing reveived data

  std::mutex scan_mutex_;                         // Protection against data races between ROS and IO threads
  std::condition_variable data_notifier_;         // Data notification condition variable

  std::deque<ScanData> scan_data_;                // Double ended queue with sucessfully received and parsed data, organized as single complete scans

  uint64_t last_data_time_;                       // time in seconds since epoch, when last data was received
};

}

