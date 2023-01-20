#ifndef ORD_SDK_DRIVER_IMPL_H_
#define ORD_SDK_DRIVER_IMPL_H_

#include <deque>
#include <condition_variable>
#include "ord/lidar_address.h"
#include "ord/ord_driver_net.h"
#include "ord/ord_types.h"

namespace ord_sdk
{

class Impl
{
public:
  Impl();

  void setTimeout(int timeout);
  error_t open(const SocketAddress& sensor);
  void close();
  bool isOpened() const;
  void GnerateProtocolFrame(uint16_t ctrl_command,std::vector<uint8_t>& frame_data,std::vector<uint8_t> content);
  error_t DealProtocolData(std::vector<uint8_t> request,std::vector<uint8_t>& response);
  error_t waitScanData(std::vector<uint8_t>& scan_block_data);

private:
  void SortingMessage(std::vector<uint8_t> message);

private:
  static const int DEFAULT_TIMEOUT = 3000;
  static const int SCAN_BLOCK_BUFFERING_COUNT = 32;

private:
  int timeout_;
  std::unique_ptr<Net> net_;
  uint16_t current_command_;
  std::mutex command_mutex_;
  std::deque<std::vector<uint8_t >> response_queue_;
  std::mutex response_queue_mutex_;
  std::condition_variable response_queue_cv_;
  std::deque<std::vector<uint8_t >> scan_block_queue_;
  std::mutex scan_block_queue_mutex_;
  std::condition_variable scan_block_queue_cv_;
};

}

#endif
