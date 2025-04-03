// Generated by gencpp from file all_om/point.msg
// DO NOT EDIT!


#ifndef ALL_OM_MESSAGE_POINT_H
#define ALL_OM_MESSAGE_POINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace all_om
{
template <class ContainerAllocator>
struct point_
{
  typedef point_<ContainerAllocator> Type;

  point_()
    : index(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , distance(0.0)
    , degree(0.0)
    , x_size(0)
    , y_size(0)
    , object()  {
    }
  point_(const ContainerAllocator& _alloc)
    : index(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , distance(0.0)
    , degree(0.0)
    , x_size(0)
    , y_size(0)
    , object(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _distance_type;
  _distance_type distance;

   typedef float _degree_type;
  _degree_type degree;

   typedef int32_t _x_size_type;
  _x_size_type x_size;

   typedef int32_t _y_size_type;
  _y_size_type y_size;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _object_type;
  _object_type object;





  typedef boost::shared_ptr< ::all_om::point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::all_om::point_<ContainerAllocator> const> ConstPtr;

}; // struct point_

typedef ::all_om::point_<std::allocator<void> > point;

typedef boost::shared_ptr< ::all_om::point > pointPtr;
typedef boost::shared_ptr< ::all_om::point const> pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::all_om::point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::all_om::point_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::all_om::point_<ContainerAllocator1> & lhs, const ::all_om::point_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.distance == rhs.distance &&
    lhs.degree == rhs.degree &&
    lhs.x_size == rhs.x_size &&
    lhs.y_size == rhs.y_size &&
    lhs.object == rhs.object;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::all_om::point_<ContainerAllocator1> & lhs, const ::all_om::point_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace all_om

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::all_om::point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::all_om::point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::all_om::point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::all_om::point_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::all_om::point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::all_om::point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::all_om::point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0af1c687bcb24ea84d8c64d9f6633331";
  }

  static const char* value(const ::all_om::point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0af1c687bcb24ea8ULL;
  static const uint64_t static_value2 = 0x4d8c64d9f6633331ULL;
};

template<class ContainerAllocator>
struct DataType< ::all_om::point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "all_om/point";
  }

  static const char* value(const ::all_om::point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::all_om::point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 index\n"
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

  static const char* value(const ::all_om::point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::all_om::point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.distance);
      stream.next(m.degree);
      stream.next(m.x_size);
      stream.next(m.y_size);
      stream.next(m.object);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::all_om::point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::all_om::point_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
    s << indent << "degree: ";
    Printer<float>::stream(s, indent + "  ", v.degree);
    s << indent << "x_size: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x_size);
    s << indent << "y_size: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y_size);
    s << indent << "object: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.object);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ALL_OM_MESSAGE_POINT_H
