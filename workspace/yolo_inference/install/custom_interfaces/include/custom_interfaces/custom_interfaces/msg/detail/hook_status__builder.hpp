// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/HookStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__HOOK_STATUS__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__HOOK_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/hook_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_HookStatus_is_hooked
{
public:
  explicit Init_HookStatus_is_hooked(::custom_interfaces::msg::HookStatus & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::HookStatus is_hooked(::custom_interfaces::msg::HookStatus::_is_hooked_type arg)
  {
    msg_.is_hooked = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::HookStatus msg_;
};

class Init_HookStatus_header
{
public:
  Init_HookStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HookStatus_is_hooked header(::custom_interfaces::msg::HookStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HookStatus_is_hooked(msg_);
  }

private:
  ::custom_interfaces::msg::HookStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::HookStatus>()
{
  return custom_interfaces::msg::builder::Init_HookStatus_header();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__HOOK_STATUS__BUILDER_HPP_
