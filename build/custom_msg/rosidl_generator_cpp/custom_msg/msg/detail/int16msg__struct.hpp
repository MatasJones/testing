// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msg:msg/Int16msg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__INT16MSG__STRUCT_HPP_
#define CUSTOM_MSG__MSG__DETAIL__INT16MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msg__msg__Int16msg __attribute__((deprecated))
#else
# define DEPRECATED__custom_msg__msg__Int16msg __declspec(deprecated)
#endif

namespace custom_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int16msg_
{
  using Type = Int16msg_<ContainerAllocator>;

  explicit Int16msg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0;
    }
  }

  explicit Int16msg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0;
    }
  }

  // field types and members
  using _num_type =
    int16_t;
  _num_type num;

  // setters for named parameter idiom
  Type & set__num(
    const int16_t & _arg)
  {
    this->num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msg::msg::Int16msg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msg::msg::Int16msg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msg::msg::Int16msg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msg::msg::Int16msg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::Int16msg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::Int16msg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::Int16msg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::Int16msg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msg::msg::Int16msg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msg::msg::Int16msg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msg__msg__Int16msg
    std::shared_ptr<custom_msg::msg::Int16msg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msg__msg__Int16msg
    std::shared_ptr<custom_msg::msg::Int16msg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int16msg_ & other) const
  {
    if (this->num != other.num) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int16msg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int16msg_

// alias to use template instance with default allocator
using Int16msg =
  custom_msg::msg::Int16msg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__INT16MSG__STRUCT_HPP_
