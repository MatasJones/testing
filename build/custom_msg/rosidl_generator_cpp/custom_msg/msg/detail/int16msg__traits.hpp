// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msg:msg/Int16msg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__INT16MSG__TRAITS_HPP_
#define CUSTOM_MSG__MSG__DETAIL__INT16MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msg/msg/detail/int16msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Int16msg & msg,
  std::ostream & out)
{
  out << "{";
  // member: num
  {
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Int16msg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Int16msg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msg

namespace rosidl_generator_traits
{

[[deprecated("use custom_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msg::msg::Int16msg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msg::msg::Int16msg & msg)
{
  return custom_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msg::msg::Int16msg>()
{
  return "custom_msg::msg::Int16msg";
}

template<>
inline const char * name<custom_msg::msg::Int16msg>()
{
  return "custom_msg/msg/Int16msg";
}

template<>
struct has_fixed_size<custom_msg::msg::Int16msg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msg::msg::Int16msg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msg::msg::Int16msg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSG__MSG__DETAIL__INT16MSG__TRAITS_HPP_
