// Generated by gencpp from file dynamixel_sdk_examples/GetVelocity.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_SDK_EXAMPLES_MESSAGE_GETVELOCITY_H
#define DYNAMIXEL_SDK_EXAMPLES_MESSAGE_GETVELOCITY_H

#include <ros/service_traits.h>


#include <dynamixel_sdk_examples/GetVelocityRequest.h>
#include <dynamixel_sdk_examples/GetVelocityResponse.h>


namespace dynamixel_sdk_examples
{

struct GetVelocity
{

typedef GetVelocityRequest Request;
typedef GetVelocityResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetVelocity
} // namespace dynamixel_sdk_examples


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamixel_sdk_examples::GetVelocity > {
  static const char* value()
  {
    return "b532ace3b383dc4c9e64687156423ac0";
  }

  static const char* value(const ::dynamixel_sdk_examples::GetVelocity&) { return value(); }
};

template<>
struct DataType< ::dynamixel_sdk_examples::GetVelocity > {
  static const char* value()
  {
    return "dynamixel_sdk_examples/GetVelocity";
  }

  static const char* value(const ::dynamixel_sdk_examples::GetVelocity&) { return value(); }
};


// service_traits::MD5Sum< ::dynamixel_sdk_examples::GetVelocityRequest> should match
// service_traits::MD5Sum< ::dynamixel_sdk_examples::GetVelocity >
template<>
struct MD5Sum< ::dynamixel_sdk_examples::GetVelocityRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_sdk_examples::GetVelocity >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::GetVelocityRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_sdk_examples::GetVelocityRequest> should match
// service_traits::DataType< ::dynamixel_sdk_examples::GetVelocity >
template<>
struct DataType< ::dynamixel_sdk_examples::GetVelocityRequest>
{
  static const char* value()
  {
    return DataType< ::dynamixel_sdk_examples::GetVelocity >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::GetVelocityRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dynamixel_sdk_examples::GetVelocityResponse> should match
// service_traits::MD5Sum< ::dynamixel_sdk_examples::GetVelocity >
template<>
struct MD5Sum< ::dynamixel_sdk_examples::GetVelocityResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_sdk_examples::GetVelocity >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::GetVelocityResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_sdk_examples::GetVelocityResponse> should match
// service_traits::DataType< ::dynamixel_sdk_examples::GetVelocity >
template<>
struct DataType< ::dynamixel_sdk_examples::GetVelocityResponse>
{
  static const char* value()
  {
    return DataType< ::dynamixel_sdk_examples::GetVelocity >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::GetVelocityResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIXEL_SDK_EXAMPLES_MESSAGE_GETVELOCITY_H
