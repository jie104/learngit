#ifndef ORD_SDK_DRIVER_NET_H_
#define ORD_SDK_DRIVER_NET_H_

#include <functional>
#include <vector>
#include <memory>
#include "ord/lidar_address.h"
#include "ord/ord_types.h"

namespace ord_sdk
{

class Net
{
public:
  const static int MESSAGE_LENGTH_MAX = 65535;
  const static int PACKET_LENGTH_MAX = 65535;

public:
  typedef std::function<void(std::vector<uint8_t>)> ReceivedMessageCallback;
  static std::unique_ptr<Net> create(const SocketAddress& sensor);

public:
  virtual ~Net() = default;

  virtual error_t open() = 0;
  virtual void close() = 0;
  virtual bool isOpened() const = 0;
  virtual void TransmitSubmit(std::vector<uint8_t> message) = 0;
  void setReceivedMessageCallback(ReceivedMessageCallback callback);

protected:
  ReceivedMessageCallback received_message_callback_;

};

}

#endif
