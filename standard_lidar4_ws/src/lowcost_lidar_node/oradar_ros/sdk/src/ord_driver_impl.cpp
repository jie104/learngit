#include "ord/ord_driver_impl.h"
#include "ord/utils.h"
#include <algorithm>
#include <cstring>

namespace ord_sdk
{

Impl::Impl()
  : timeout_(DEFAULT_TIMEOUT)
{
}

void Impl::setTimeout(int timeout)
{
  timeout_ = timeout;
}

error_t Impl::open(const SocketAddress& sensor)
{
  net_ = Net::create(sensor);
  net_->setReceivedMessageCallback(std::bind(&Impl::SortingMessage, this, std::placeholders::_1));
  return net_->open();
}


void Impl::close()
{
  if (net_) {
    if (net_->isOpened())
      net_->close();
    net_ = nullptr;
  }
}

bool Impl::isOpened() const
{
  return (net_ && net_->isOpened());
}

void Impl::GnerateProtocolFrame(uint16_t ctrl_command,std::vector<uint8_t>& frame_data,std::vector<uint8_t> content)
{
  ordFrameHead framehead;
  uint32_t CRCResult = 0;
  uint32_t framelenth = sizeof(ordFrameHead)+content.size()+sizeof(CRCResult);
  frame_data.clear(); 
  memset(&framehead,0,sizeof(framehead));
  framehead.ctrl_un.ctrlcomm = htons(ctrl_command);
  current_command_ = framehead.ctrl_un.ctrlcomm;
  framehead.framelenth = htons(framelenth);
  frame_data.insert(frame_data.begin(),(uint8_t*)&framehead,(uint8_t*)&framehead+sizeof(framehead));
  frame_data.insert(frame_data.begin()+sizeof(framehead),content.begin(),content.end());
  CRCResult = htonl(Utils::calcCrc32(frame_data.data(),framelenth-4));
  frame_data.insert((frame_data.begin() + framelenth - 4),(uint8_t*)&CRCResult,(uint8_t*)&CRCResult+sizeof(CRCResult));
}

error_t Impl::DealProtocolData(std::vector<uint8_t> request,std::vector<uint8_t>& response)
{
    if(timeout_ == 0)
    {
      return timed_out;
    }
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::unique_lock<std::mutex> response_queue_lock(response_queue_mutex_);

    net_->TransmitSubmit(std::move(request));

    bool wait_result = response_queue_cv_.wait_for(response_queue_lock, std::chrono::milliseconds(timeout_), [&]() {
      std::remove_if(response_queue_.begin(), response_queue_.end(), [&](const std::vector<uint8_t>& document) {
        return (*(uint16_t*)&document[6] != current_command_);
      });
      return (response_queue_.size() > 0 && *(uint16_t*)(&response_queue_.front()[6]) == current_command_);
    });
    if(wait_result){
      std::vector<uint8_t>& message = response_queue_.front();
      ordFrameHead* pframehead =  reinterpret_cast<ordFrameHead*>(message.data());
      uint32_t operateResult = ntohl(*(uint32_t*)(message.data() + sizeof(ordFrameHead)));
      if(pframehead->resp_un.resp_str.type == 0x01){//设置应答
        if(operateResult == 0x00){//设置成功
          return no_error;
        }else//设置失败
        {
          response.resize(4);
          *(uint32_t*) response.data() = operateResult;
          return operation_failure;
        }
      }if(pframehead->resp_un.resp_str.type == 0x02){//查询应答
        uint32_t queryResponeResult = ntohl(*(uint32_t*)&message[sizeof(ordFrameHead)+sizeof(operateResult)]);
        if(operateResult == 0x00){//查询成功
          switch (ntohs(pframehead->resp_un.respcomm))
          {
          case get_lidar_ip:
          case get_lidar_port:
          case get_filter_level:
          case get_timestamp:
          case get_sync_status:
          case get_scan_speed:
          case get_motor_speed:
          case get_temperature:
          case get_high_voltage:
          case get_scan_direction:
            response.resize(4);
            response.assign((message.begin() + sizeof(ordFrameHead) + sizeof(operateResult)) , (message.begin() + sizeof(ordFrameHead) + sizeof(operateResult) + response.size()));
            break;
          case get_hardware_version:
          case get_firmware_version:
            response.resize(16);
            response.assign((message.begin() + sizeof(ordFrameHead) + sizeof(operateResult)) , (message.begin() + sizeof(ordFrameHead) + sizeof(operateResult) + response.size()));              
            break;
          default:
            response.resize(4);
            *(uint32_t*) response.data() = queryResponeResult;
            break;
          }     
          return no_error;
        }else if(operateResult == 0x01)//查询失败
        {
          response.resize(4);
          *(uint32_t*) response.data() = queryResponeResult;
           return operation_failure;
        }      
      }

    }else{
      return error_t::timed_out;
    }
  return no_error;
}

error_t Impl::waitScanData(std::vector<uint8_t>& scan_block_data)
{
  if(timeout_ == 0)
  {
    return timed_out;
  }
  
  std::unique_lock<std::mutex> scan_block_queue_lock(scan_block_queue_mutex_);
  bool wait_result = scan_block_queue_cv_.wait_for(scan_block_queue_lock, std::chrono::milliseconds(timeout_), [&]() {
    return (scan_block_queue_.size() > 0);
  });
  if (wait_result) {
    if (scan_block_queue_.size() > 0) {
      scan_block_data = std::move(scan_block_queue_.front());
      scan_block_queue_.pop_front();
    }
    return error_t::no_error;
  }
  else
    return error_t::timed_out;
}

void Impl::SortingMessage(std::vector<uint8_t> message)
{
  ordFrameHead* pframehead =  reinterpret_cast<ordFrameHead*>(message.data());
  switch(pframehead->frametype)
  {
      case 0x00:
      {
        std::lock_guard<std::mutex> response_queue_lock(response_queue_mutex_);
        if (pframehead->resp_un.respcomm == current_command_) {
          response_queue_.clear();
          response_queue_.push_back(std::move(message));
          response_queue_cv_.notify_one();
        }
        break;
      }
      case 0x01:
      {
        std::lock_guard<std::mutex> scan_block_queue_lock(scan_block_queue_mutex_);
        if (scan_block_queue_.size() == SCAN_BLOCK_BUFFERING_COUNT)
          scan_block_queue_.pop_front();
        scan_block_queue_.push_back(std::move(message));
        scan_block_queue_cv_.notify_one();   
        break;
      }
      default :
      {
        break;
      }
  }
}

}
