// Generated by gencpp from file phidgets_msgs/SetAnalogOutput.msg
// DO NOT EDIT!


#ifndef PHIDGETS_MSGS_MESSAGE_SETANALOGOUTPUT_H
#define PHIDGETS_MSGS_MESSAGE_SETANALOGOUTPUT_H

#include <ros/service_traits.h>


#include <phidgets_msgs/SetAnalogOutputRequest.h>
#include <phidgets_msgs/SetAnalogOutputResponse.h>


namespace phidgets_msgs
{

struct SetAnalogOutput
{

typedef SetAnalogOutputRequest Request;
typedef SetAnalogOutputResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetAnalogOutput
} // namespace phidgets_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::phidgets_msgs::SetAnalogOutput > {
  static const char* value()
  {
    return "c0d7b329e28c7be8f18cb4c1bd42580f";
  }

  static const char* value(const ::phidgets_msgs::SetAnalogOutput&) { return value(); }
};

template<>
struct DataType< ::phidgets_msgs::SetAnalogOutput > {
  static const char* value()
  {
    return "phidgets_msgs/SetAnalogOutput";
  }

  static const char* value(const ::phidgets_msgs::SetAnalogOutput&) { return value(); }
};


// service_traits::MD5Sum< ::phidgets_msgs::SetAnalogOutputRequest> should match
// service_traits::MD5Sum< ::phidgets_msgs::SetAnalogOutput >
template<>
struct MD5Sum< ::phidgets_msgs::SetAnalogOutputRequest>
{
  static const char* value()
  {
    return MD5Sum< ::phidgets_msgs::SetAnalogOutput >::value();
  }
  static const char* value(const ::phidgets_msgs::SetAnalogOutputRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::phidgets_msgs::SetAnalogOutputRequest> should match
// service_traits::DataType< ::phidgets_msgs::SetAnalogOutput >
template<>
struct DataType< ::phidgets_msgs::SetAnalogOutputRequest>
{
  static const char* value()
  {
    return DataType< ::phidgets_msgs::SetAnalogOutput >::value();
  }
  static const char* value(const ::phidgets_msgs::SetAnalogOutputRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::phidgets_msgs::SetAnalogOutputResponse> should match
// service_traits::MD5Sum< ::phidgets_msgs::SetAnalogOutput >
template<>
struct MD5Sum< ::phidgets_msgs::SetAnalogOutputResponse>
{
  static const char* value()
  {
    return MD5Sum< ::phidgets_msgs::SetAnalogOutput >::value();
  }
  static const char* value(const ::phidgets_msgs::SetAnalogOutputResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::phidgets_msgs::SetAnalogOutputResponse> should match
// service_traits::DataType< ::phidgets_msgs::SetAnalogOutput >
template<>
struct DataType< ::phidgets_msgs::SetAnalogOutputResponse>
{
  static const char* value()
  {
    return DataType< ::phidgets_msgs::SetAnalogOutput >::value();
  }
  static const char* value(const ::phidgets_msgs::SetAnalogOutputResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PHIDGETS_MSGS_MESSAGE_SETANALOGOUTPUT_H