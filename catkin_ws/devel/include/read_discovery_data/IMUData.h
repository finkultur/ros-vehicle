/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/faj/catkin_ws/src/read_discovery_data/msg/IMUData.msg
 *
 */


#ifndef READ_DISCOVERY_DATA_MESSAGE_IMUDATA_H
#define READ_DISCOVERY_DATA_MESSAGE_IMUDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace read_discovery_data
{
template <class ContainerAllocator>
struct IMUData_
{
  typedef IMUData_<ContainerAllocator> Type;

  IMUData_()
    : gyro0(0.0)
    , gyro1(0.0)
    , gyro2(0.0)
    , acc0(0.0)
    , acc1(0.0)
    , acc2(0.0)
    , mag0(0.0)
    , mag1(0.0)
    , mag2(0.0)  {
    }
  IMUData_(const ContainerAllocator& _alloc)
    : gyro0(0.0)
    , gyro1(0.0)
    , gyro2(0.0)
    , acc0(0.0)
    , acc1(0.0)
    , acc2(0.0)
    , mag0(0.0)
    , mag1(0.0)
    , mag2(0.0)  {
    }



   typedef double _gyro0_type;
  _gyro0_type gyro0;

   typedef double _gyro1_type;
  _gyro1_type gyro1;

   typedef double _gyro2_type;
  _gyro2_type gyro2;

   typedef double _acc0_type;
  _acc0_type acc0;

   typedef double _acc1_type;
  _acc1_type acc1;

   typedef double _acc2_type;
  _acc2_type acc2;

   typedef double _mag0_type;
  _mag0_type mag0;

   typedef double _mag1_type;
  _mag1_type mag1;

   typedef double _mag2_type;
  _mag2_type mag2;




  typedef boost::shared_ptr< ::read_discovery_data::IMUData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::read_discovery_data::IMUData_<ContainerAllocator> const> ConstPtr;

}; // struct IMUData_

typedef ::read_discovery_data::IMUData_<std::allocator<void> > IMUData;

typedef boost::shared_ptr< ::read_discovery_data::IMUData > IMUDataPtr;
typedef boost::shared_ptr< ::read_discovery_data::IMUData const> IMUDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::read_discovery_data::IMUData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::read_discovery_data::IMUData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace read_discovery_data

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'read_discovery_data': ['/home/faj/catkin_ws/src/read_discovery_data/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::read_discovery_data::IMUData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::read_discovery_data::IMUData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::read_discovery_data::IMUData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::read_discovery_data::IMUData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::read_discovery_data::IMUData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::read_discovery_data::IMUData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::read_discovery_data::IMUData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e078f0b1cfbbc42a2946ba147a6b6218";
  }

  static const char* value(const ::read_discovery_data::IMUData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe078f0b1cfbbc42aULL;
  static const uint64_t static_value2 = 0x2946ba147a6b6218ULL;
};

template<class ContainerAllocator>
struct DataType< ::read_discovery_data::IMUData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "read_discovery_data/IMUData";
  }

  static const char* value(const ::read_discovery_data::IMUData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::read_discovery_data::IMUData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 gyro0\n\
float64 gyro1\n\
float64 gyro2\n\
float64 acc0\n\
float64 acc1\n\
float64 acc2\n\
float64 mag0\n\
float64 mag1\n\
float64 mag2\n\
";
  }

  static const char* value(const ::read_discovery_data::IMUData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::read_discovery_data::IMUData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gyro0);
      stream.next(m.gyro1);
      stream.next(m.gyro2);
      stream.next(m.acc0);
      stream.next(m.acc1);
      stream.next(m.acc2);
      stream.next(m.mag0);
      stream.next(m.mag1);
      stream.next(m.mag2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct IMUData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::read_discovery_data::IMUData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::read_discovery_data::IMUData_<ContainerAllocator>& v)
  {
    s << indent << "gyro0: ";
    Printer<double>::stream(s, indent + "  ", v.gyro0);
    s << indent << "gyro1: ";
    Printer<double>::stream(s, indent + "  ", v.gyro1);
    s << indent << "gyro2: ";
    Printer<double>::stream(s, indent + "  ", v.gyro2);
    s << indent << "acc0: ";
    Printer<double>::stream(s, indent + "  ", v.acc0);
    s << indent << "acc1: ";
    Printer<double>::stream(s, indent + "  ", v.acc1);
    s << indent << "acc2: ";
    Printer<double>::stream(s, indent + "  ", v.acc2);
    s << indent << "mag0: ";
    Printer<double>::stream(s, indent + "  ", v.mag0);
    s << indent << "mag1: ";
    Printer<double>::stream(s, indent + "  ", v.mag1);
    s << indent << "mag2: ";
    Printer<double>::stream(s, indent + "  ", v.mag2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // READ_DISCOVERY_DATA_MESSAGE_IMUDATA_H
