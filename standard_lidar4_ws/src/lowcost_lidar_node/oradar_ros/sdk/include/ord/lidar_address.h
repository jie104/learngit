#ifndef ORD_SDK_LIDAR_ADDRESS_H_
#define ORD_SDK_LIDAR_ADDRESS_H_

#ifdef __linux__
#include <netinet/in.h>
#elif _WIN32
#include <WinSock2.h>
typedef ULONG in_addr_t;
typedef USHORT in_port_t;
#endif

namespace ord_sdk
{

class SocketAddress
{
public:
  virtual ~SocketAddress() = default;

protected:
  SocketAddress() = default;
};

class LidarAddress : public SocketAddress
{
public:
  LidarAddress(in_addr_t address, in_port_t port);
  LidarAddress(const LidarAddress&) = default;

  LidarAddress& operator=(const LidarAddress&) = default;
  bool operator==(const LidarAddress& other) const;

  in_addr_t address() const;
  in_port_t port() const;

private:
  in_addr_t address_;
  in_port_t port_;
};

}

#endif
