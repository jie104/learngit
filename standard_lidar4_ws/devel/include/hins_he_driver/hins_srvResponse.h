// Generated by gencpp from file hins_he_driver/hins_srvResponse.msg
// DO NOT EDIT!


#ifndef HINS_HE_DRIVER_MESSAGE_HINS_SRVRESPONSE_H
#define HINS_HE_DRIVER_MESSAGE_HINS_SRVRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hins_he_driver
{
template <class ContainerAllocator>
struct hins_srvResponse_
{
  typedef hins_srvResponse_<ContainerAllocator> Type;

  hins_srvResponse_()
    : area1(false)
    , area2(false)
    , area3(false)
    , success(false)  {
    }
  hins_srvResponse_(const ContainerAllocator& _alloc)
    : area1(false)
    , area2(false)
    , area3(false)
    , success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _area1_type;
  _area1_type area1;

   typedef uint8_t _area2_type;
  _area2_type area2;

   typedef uint8_t _area3_type;
  _area3_type area3;

   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct hins_srvResponse_

typedef ::hins_he_driver::hins_srvResponse_<std::allocator<void> > hins_srvResponse;

typedef boost::shared_ptr< ::hins_he_driver::hins_srvResponse > hins_srvResponsePtr;
typedef boost::shared_ptr< ::hins_he_driver::hins_srvResponse const> hins_srvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hins_he_driver::hins_srvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hins_he_driver::hins_srvResponse_<ContainerAllocator1> & lhs, const ::hins_he_driver::hins_srvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.area1 == rhs.area1 &&
    lhs.area2 == rhs.area2 &&
    lhs.area3 == rhs.area3 &&
    lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hins_he_driver::hins_srvResponse_<ContainerAllocator1> & lhs, const ::hins_he_driver::hins_srvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hins_he_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5fd013987c49e2331580b63f581e6836";
  }

  static const char* value(const ::hins_he_driver::hins_srvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5fd013987c49e233ULL;
  static const uint64_t static_value2 = 0x1580b63f581e6836ULL;
};

template<class ContainerAllocator>
struct DataType< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hins_he_driver/hins_srvResponse";
  }

  static const char* value(const ::hins_he_driver::hins_srvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool area1\n"
"bool area2\n"
"bool area3\n"
"bool success\n"
"\n"
;
  }

  static const char* value(const ::hins_he_driver::hins_srvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.area1);
      stream.next(m.area2);
      stream.next(m.area3);
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct hins_srvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hins_he_driver::hins_srvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hins_he_driver::hins_srvResponse_<ContainerAllocator>& v)
  {
    s << indent << "area1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.area1);
    s << indent << "area2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.area2);
    s << indent << "area3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.area3);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HINS_HE_DRIVER_MESSAGE_HINS_SRVRESPONSE_H
