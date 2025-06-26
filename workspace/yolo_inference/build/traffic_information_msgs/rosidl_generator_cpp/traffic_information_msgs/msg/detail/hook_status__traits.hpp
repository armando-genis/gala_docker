// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traffic_information_msgs:msg/HookStatus.idl
// generated code does not contain a copyright notice

#ifndef TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__TRAITS_HPP_
#define TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "traffic_information_msgs/msg/detail/hook_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace traffic_information_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HookStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_hooked
  {
    out << "is_hooked: ";
    rosidl_generator_traits::value_to_yaml(msg.is_hooked, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HookStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: is_hooked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_hooked: ";
    rosidl_generator_traits::value_to_yaml(msg.is_hooked, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HookStatus & msg, bool use_flow_style = false)
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

}  // namespace traffic_information_msgs

namespace rosidl_generator_traits
{

[[deprecated("use traffic_information_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const traffic_information_msgs::msg::HookStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  traffic_information_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use traffic_information_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const traffic_information_msgs::msg::HookStatus & msg)
{
  return traffic_information_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<traffic_information_msgs::msg::HookStatus>()
{
  return "traffic_information_msgs::msg::HookStatus";
}

template<>
inline const char * name<traffic_information_msgs::msg::HookStatus>()
{
  return "traffic_information_msgs/msg/HookStatus";
}

template<>
struct has_fixed_size<traffic_information_msgs::msg::HookStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<traffic_information_msgs::msg::HookStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<traffic_information_msgs::msg::HookStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__TRAITS_HPP_
