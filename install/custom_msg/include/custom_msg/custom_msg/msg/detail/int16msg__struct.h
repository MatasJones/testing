// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/Int16msg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__INT16MSG__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__INT16MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Int16msg in the package custom_msg.
/**
  * Int16msg
 */
typedef struct custom_msg__msg__Int16msg
{
  int16_t num;
} custom_msg__msg__Int16msg;

// Struct for a sequence of custom_msg__msg__Int16msg.
typedef struct custom_msg__msg__Int16msg__Sequence
{
  custom_msg__msg__Int16msg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__Int16msg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__INT16MSG__STRUCT_H_
