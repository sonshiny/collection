// Generated by gencpp from file all_om/points.msg
// DO NOT EDIT!


#ifndef ALL_OM_MESSAGE_POINTS_H
#define ALL_OM_MESSAGE_POINTS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <all_om/point.h>

namespace all_om
{
template <class ContainerAllocator>
struct points_
{
  typedef points_<ContainerAllocator> Type;

  points_()
    : num()
    , s(0)
    , data()  {
    }
  points_(const ContainerAllocator& _alloc)
    : num(_alloc)
    , s(0)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _num_type;
  _num_type num;

   typedef int32_t _s_type;
  _s_type s;

   typedef std::vector< ::all_om::point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::all_om::point_<ContainerAllocator> >> _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::all_om::points_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::all_om::points_<ContainerAllocator> const> ConstPtr;

}; // struct points_

typedef ::all_om::points_<std::allocator<void> > points;

typedef boost::shared_ptr< ::all_om::points > pointsPtr;
typedef boost::shared_ptr< ::all_om::points const> pointsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::all_om::points_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::all_om::points_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::all_om::points_<ContainerAllocator1> & lhs, const ::all_om::points_<ContainerAllocator2> & rhs)
{
  return lhs.num == rhs.num &&
    lhs.s == rhs.s &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::all_om::points_<ContainerAllocator1> & lhs, const ::all_om::points_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace all_om

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::all_om::points_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::all_om::points_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::all_om::points_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::all_om::points_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::all_om::points_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::all_om::points_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::all_om::points_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bc1d3270b5a8918bda079385939e1a51";
  }

  static const char* value(const ::all_om::points_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbc1d3270b5a8918bULL;
  static const uint64_t static_value2 = 0xda079385939e1a51ULL;
};

template<class ContainerAllocator>
struct DataType< ::all_om::points_<ContainerAllocator> >
{
  static const char* value()
  {
    return "all_om/points";
  }

  static const char* value(const ::all_om::points_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::all_om::points_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string num\n"
"int32 s\n"
"point[] data\n"
"\n"
"================================================================================\n"
"MSG: all_om/point\n"
"int32 index\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 distance\n"
"float32 degree\n"
"int32 x_size\n"
"int32 y_size\n"
"string object\n"
;
  }

  static const char* value(const ::all_om::points_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::all_om::points_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num);
      stream.next(m.s);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct points_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::all_om::points_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::all_om::points_<ContainerAllocator>& v)
  {
    s << indent << "num: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.num);
    s << indent << "s: ";
    Printer<int32_t>::stream(s, indent + "  ", v.s);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::all_om::point_<ContainerAllocator> >::stream(s, indent + "    ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ALL_OM_MESSAGE_POINTS_H
