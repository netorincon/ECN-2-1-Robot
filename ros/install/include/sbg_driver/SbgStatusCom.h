// Generated by gencpp from file sbg_driver/SbgStatusCom.msg
// DO NOT EDIT!


#ifndef SBG_DRIVER_MESSAGE_SBGSTATUSCOM_H
#define SBG_DRIVER_MESSAGE_SBGSTATUSCOM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sbg_driver
{
template <class ContainerAllocator>
struct SbgStatusCom_
{
  typedef SbgStatusCom_<ContainerAllocator> Type;

  SbgStatusCom_()
    : port_a(false)
    , port_b(false)
    , port_c(false)
    , port_d(false)
    , port_e(false)
    , port_a_rx(false)
    , port_a_tx(false)
    , port_b_rx(false)
    , port_b_tx(false)
    , port_c_rx(false)
    , port_c_tx(false)
    , port_d_rx(false)
    , port_d_tx(false)
    , port_e_rx(false)
    , port_e_tx(false)
    , can_rx(false)
    , can_tx(false)
    , can_status(0)  {
    }
  SbgStatusCom_(const ContainerAllocator& _alloc)
    : port_a(false)
    , port_b(false)
    , port_c(false)
    , port_d(false)
    , port_e(false)
    , port_a_rx(false)
    , port_a_tx(false)
    , port_b_rx(false)
    , port_b_tx(false)
    , port_c_rx(false)
    , port_c_tx(false)
    , port_d_rx(false)
    , port_d_tx(false)
    , port_e_rx(false)
    , port_e_tx(false)
    , can_rx(false)
    , can_tx(false)
    , can_status(0)  {
  (void)_alloc;
    }



   typedef uint8_t _port_a_type;
  _port_a_type port_a;

   typedef uint8_t _port_b_type;
  _port_b_type port_b;

   typedef uint8_t _port_c_type;
  _port_c_type port_c;

   typedef uint8_t _port_d_type;
  _port_d_type port_d;

   typedef uint8_t _port_e_type;
  _port_e_type port_e;

   typedef uint8_t _port_a_rx_type;
  _port_a_rx_type port_a_rx;

   typedef uint8_t _port_a_tx_type;
  _port_a_tx_type port_a_tx;

   typedef uint8_t _port_b_rx_type;
  _port_b_rx_type port_b_rx;

   typedef uint8_t _port_b_tx_type;
  _port_b_tx_type port_b_tx;

   typedef uint8_t _port_c_rx_type;
  _port_c_rx_type port_c_rx;

   typedef uint8_t _port_c_tx_type;
  _port_c_tx_type port_c_tx;

   typedef uint8_t _port_d_rx_type;
  _port_d_rx_type port_d_rx;

   typedef uint8_t _port_d_tx_type;
  _port_d_tx_type port_d_tx;

   typedef uint8_t _port_e_rx_type;
  _port_e_rx_type port_e_rx;

   typedef uint8_t _port_e_tx_type;
  _port_e_tx_type port_e_tx;

   typedef uint8_t _can_rx_type;
  _can_rx_type can_rx;

   typedef uint8_t _can_tx_type;
  _can_tx_type can_tx;

   typedef uint8_t _can_status_type;
  _can_status_type can_status;





  typedef boost::shared_ptr< ::sbg_driver::SbgStatusCom_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sbg_driver::SbgStatusCom_<ContainerAllocator> const> ConstPtr;

}; // struct SbgStatusCom_

typedef ::sbg_driver::SbgStatusCom_<std::allocator<void> > SbgStatusCom;

typedef boost::shared_ptr< ::sbg_driver::SbgStatusCom > SbgStatusComPtr;
typedef boost::shared_ptr< ::sbg_driver::SbgStatusCom const> SbgStatusComConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sbg_driver::SbgStatusCom_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sbg_driver::SbgStatusCom_<ContainerAllocator1> & lhs, const ::sbg_driver::SbgStatusCom_<ContainerAllocator2> & rhs)
{
  return lhs.port_a == rhs.port_a &&
    lhs.port_b == rhs.port_b &&
    lhs.port_c == rhs.port_c &&
    lhs.port_d == rhs.port_d &&
    lhs.port_e == rhs.port_e &&
    lhs.port_a_rx == rhs.port_a_rx &&
    lhs.port_a_tx == rhs.port_a_tx &&
    lhs.port_b_rx == rhs.port_b_rx &&
    lhs.port_b_tx == rhs.port_b_tx &&
    lhs.port_c_rx == rhs.port_c_rx &&
    lhs.port_c_tx == rhs.port_c_tx &&
    lhs.port_d_rx == rhs.port_d_rx &&
    lhs.port_d_tx == rhs.port_d_tx &&
    lhs.port_e_rx == rhs.port_e_rx &&
    lhs.port_e_tx == rhs.port_e_tx &&
    lhs.can_rx == rhs.can_rx &&
    lhs.can_tx == rhs.can_tx &&
    lhs.can_status == rhs.can_status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sbg_driver::SbgStatusCom_<ContainerAllocator1> & lhs, const ::sbg_driver::SbgStatusCom_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sbg_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sbg_driver::SbgStatusCom_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sbg_driver::SbgStatusCom_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sbg_driver::SbgStatusCom_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0586194daf83121bc54eda7bece4880f";
  }

  static const char* value(const ::sbg_driver::SbgStatusCom_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0586194daf83121bULL;
  static const uint64_t static_value2 = 0xc54eda7bece4880fULL;
};

template<class ContainerAllocator>
struct DataType< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sbg_driver/SbgStatusCom";
  }

  static const char* value(const ::sbg_driver::SbgStatusCom_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# SBG Ellipse Messages\n"
"# SbgStatus submessage\n"
"\n"
"# PORT A: False in case of low level communication error.\n"
"bool port_a\n"
"\n"
"# PORT B: False in case of low level communication error.\n"
"bool port_b\n"
"\n"
"# PORT C: False in case of low level communication error.\n"
"bool port_c\n"
"\n"
"# PORT D: False in case of low level communication error.\n"
"bool port_d\n"
"\n"
"# PORT E: False in case of low level communication error.\n"
"bool port_e\n"
"\n"
"# PORT A RX: False in case of saturation on PORT A input\n"
"bool port_a_rx\n"
"\n"
"# PORT A TX: False in case of saturation on PORT A output\n"
"bool port_a_tx\n"
"\n"
"# PORT B RX: False in case of saturation on PORT B input\n"
"bool port_b_rx\n"
"\n"
"# PORT B TX: False in case of saturation on PORT B output\n"
"bool port_b_tx\n"
"\n"
"# PORT C RX: False in case of saturation on PORT C input\n"
"bool port_c_rx\n"
"\n"
"# PORT C TX: False in case of saturation on PORT C output\n"
"bool port_c_tx\n"
"\n"
"# PORT D RX: False in case of saturation on PORT D input\n"
"bool port_d_rx\n"
"\n"
"# PORT D TX: False in case of saturation on PORT D output\n"
"bool port_d_tx\n"
"\n"
"# PORT E RX: False in case of saturation on PORT E input\n"
"bool port_e_rx\n"
"\n"
"# PORT E TX: False in case of saturation on PORT E output\n"
"bool port_e_tx\n"
"\n"
"# CAN RX: False in case of saturation on CAN Bus output buffer\n"
"bool can_rx\n"
"\n"
"# CAN TX: False in case of saturation on CAN Bus input buffer\n"
"bool can_tx\n"
"\n"
"# CAN BUS\n"
"# 0 CAN BUS OFF 		Bus OFF operation due to too much errors.\n"
"# 1 CAN BUS TX_RX_ERR 	Transmit or received error.\n"
"# 2 CAN BUS OK 			The CAN bus is working correctly.\n"
"# 3 CAN BUS ERROR 		A general error has occurred on the CAN bus.\n"
"uint8 can_status\n"
;
  }

  static const char* value(const ::sbg_driver::SbgStatusCom_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.port_a);
      stream.next(m.port_b);
      stream.next(m.port_c);
      stream.next(m.port_d);
      stream.next(m.port_e);
      stream.next(m.port_a_rx);
      stream.next(m.port_a_tx);
      stream.next(m.port_b_rx);
      stream.next(m.port_b_tx);
      stream.next(m.port_c_rx);
      stream.next(m.port_c_tx);
      stream.next(m.port_d_rx);
      stream.next(m.port_d_tx);
      stream.next(m.port_e_rx);
      stream.next(m.port_e_tx);
      stream.next(m.can_rx);
      stream.next(m.can_tx);
      stream.next(m.can_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SbgStatusCom_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sbg_driver::SbgStatusCom_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sbg_driver::SbgStatusCom_<ContainerAllocator>& v)
  {
    s << indent << "port_a: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_a);
    s << indent << "port_b: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_b);
    s << indent << "port_c: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_c);
    s << indent << "port_d: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_d);
    s << indent << "port_e: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_e);
    s << indent << "port_a_rx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_a_rx);
    s << indent << "port_a_tx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_a_tx);
    s << indent << "port_b_rx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_b_rx);
    s << indent << "port_b_tx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_b_tx);
    s << indent << "port_c_rx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_c_rx);
    s << indent << "port_c_tx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_c_tx);
    s << indent << "port_d_rx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_d_rx);
    s << indent << "port_d_tx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_d_tx);
    s << indent << "port_e_rx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_e_rx);
    s << indent << "port_e_tx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.port_e_tx);
    s << indent << "can_rx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.can_rx);
    s << indent << "can_tx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.can_tx);
    s << indent << "can_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.can_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SBG_DRIVER_MESSAGE_SBGSTATUSCOM_H
