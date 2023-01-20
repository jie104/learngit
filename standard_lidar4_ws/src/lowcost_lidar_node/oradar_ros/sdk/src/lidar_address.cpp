#include "ord/lidar_address.h"

namespace ord_sdk
{

LidarAddress::LidarAddress(in_addr_t address, in_port_t port)
  : address_(address)
  , port_(port)
{
}

bool LidarAddress::operator==(const LidarAddress& other) const
{
  return (address_ == other.address_) && (port_ == other.port_);
}

in_addr_t LidarAddress::address() const
{
  return address_;
}

in_port_t LidarAddress::port() const
{
  return port_;
}

}
