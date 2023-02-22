// Generated by gencpp from file phidgets_msgs/SetAnalogOutputResponse.msg
// DO NOT EDIT!


#ifndef PHIDGETS_MSGS_MESSAGE_SETANALOGOUTPUTRESPONSE_H
#define PHIDGETS_MSGS_MESSAGE_SETANALOGOUTPUTRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace phidgets_msgs
{
template <class ContainerAllocator>
struct SetAnalogOutputResponse_
{
  typedef SetAnalogOutputResponse_<ContainerAllocator> Type;

  SetAnalogOutputResponse_()
    : success(false)  {
    }
  SetAnalogOutputResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetAnalogOutputResponse_

typedef ::phidgets_msgs::SetAnalogOutputResponse_<std::allocator<void> > SetAnalogOutputResponse;

typedef boost::shared_ptr< ::phidgets_msgs::SetAnalogOutputResponse > SetAnalogOutputResponsePtr;
typedef boost::shared_ptr< ::phidgets_msgs::SetAnalogOutputResponse const> SetAnalogOutputResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator1> & lhs, const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator1> & lhs, const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace phidgets_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phidgets_msgs/SetAnalogOutputResponse";
  }

  static const char* value(const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetAnalogOutputResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phidgets_msgs::SetAnalogOutputResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHIDGETS_MSGS_MESSAGE_SETANALOGOUTPUTRESPONSE_H