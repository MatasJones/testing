// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/Int16msg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__INT16MSG__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__INT16MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/int16msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_Int16msg_num
{
public:
  Init_Int16msg_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_msg::msg::Int16msg num(::custom_msg::msg::Int16msg::_num_type arg)
  {
    msg_.num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::Int16msg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::Int16msg>()
{
  return custom_msg::msg::builder::Init_Int16msg_num();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__INT16MSG__BUILDER_HPP_
