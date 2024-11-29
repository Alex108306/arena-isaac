// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from isaacsim_msgs:msg/Quat.idl
// generated code does not contain a copyright notice

#ifndef ISAACSIM_MSGS__MSG__DETAIL__QUAT__STRUCT_HPP_
#define ISAACSIM_MSGS__MSG__DETAIL__QUAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__isaacsim_msgs__msg__Quat __attribute__((deprecated))
#else
# define DEPRECATED__isaacsim_msgs__msg__Quat __declspec(deprecated)
#endif

namespace isaacsim_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Quat_
{
  using Type = Quat_<ContainerAllocator>;

  explicit Quat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->w = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  explicit Quat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->w = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  // field types and members
  using _w_type =
    float;
  _w_type w;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;

  // setters for named parameter idiom
  Type & set__w(
    const float & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaacsim_msgs::msg::Quat_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaacsim_msgs::msg::Quat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaacsim_msgs::msg::Quat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaacsim_msgs::msg::Quat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaacsim_msgs__msg__Quat
    std::shared_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaacsim_msgs__msg__Quat
    std::shared_ptr<isaacsim_msgs::msg::Quat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Quat_ & other) const
  {
    if (this->w != other.w) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Quat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Quat_

// alias to use template instance with default allocator
using Quat =
  isaacsim_msgs::msg::Quat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace isaacsim_msgs

#endif  // ISAACSIM_MSGS__MSG__DETAIL__QUAT__STRUCT_HPP_
