// Generated by gencpp from file dynamixel_sdk_examples/GetVelocityRequest.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_SDK_EXAMPLES_MESSAGE_GETVELOCITYREQUEST_H
#define DYNAMIXEL_SDK_EXAMPLES_MESSAGE_GETVELOCITYREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_sdk_examples
{
template <class ContainerAllocator>
struct GetVelocityRequest_
{
  typedef GetVelocityRequest_<ContainerAllocator> Type;

  GetVelocityRequest_()
    : id(0)  {
    }
  GetVelocityRequest_(const ContainerAllocator& _alloc)
    : id(0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;





  typedef boost::shared_ptr< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetVelocityRequest_

typedef ::dynamixel_sdk_examples::GetVelocityRequest_<std::allocator<void> > GetVelocityRequest;

typedef boost::shared_ptr< ::dynamixel_sdk_examples::GetVelocityRequest > GetVelocityRequestPtr;
typedef boost::shared_ptr< ::dynamixel_sdk_examples::GetVelocityRequest const> GetVelocityRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator1> & lhs, const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator1> & lhs, const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamixel_sdk_examples

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "541b98e964705918fa8eb206b65347b3";
  }

  static const char* value(const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x541b98e964705918ULL;
  static const uint64_t static_value2 = 0xfa8eb206b65347b3ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_sdk_examples/GetVelocityRequest";
  }

  static const char* value(const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id\n"
;
  }

  static const char* value(const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetVelocityRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_sdk_examples::GetVelocityRequest_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_SDK_EXAMPLES_MESSAGE_GETVELOCITYREQUEST_H
