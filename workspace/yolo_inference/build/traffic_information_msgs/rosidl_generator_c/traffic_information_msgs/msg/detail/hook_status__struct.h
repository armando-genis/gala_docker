// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from traffic_information_msgs:msg/HookStatus.idl
// generated code does not contain a copyright notice

#ifndef TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__STRUCT_H_
#define TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/HookStatus in the package traffic_information_msgs.
typedef struct traffic_information_msgs__msg__HookStatus
{
  std_msgs__msg__Header header;
  bool is_hooked;
} traffic_information_msgs__msg__HookStatus;

// Struct for a sequence of traffic_information_msgs__msg__HookStatus.
typedef struct traffic_information_msgs__msg__HookStatus__Sequence
{
  traffic_information_msgs__msg__HookStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} traffic_information_msgs__msg__HookStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__STRUCT_H_
