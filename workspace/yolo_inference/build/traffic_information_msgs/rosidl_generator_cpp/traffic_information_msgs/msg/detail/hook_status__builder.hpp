// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from traffic_information_msgs:msg/HookStatus.idl
// generated code does not contain a copyright notice

#ifndef TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__BUILDER_HPP_
#define TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "traffic_information_msgs/msg/detail/hook_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace traffic_information_msgs
{

namespace msg
{

namespace builder
{

class Init_HookStatus_is_hooked
{
public:
  explicit Init_HookStatus_is_hooked(::traffic_information_msgs::msg::HookStatus & msg)
  : msg_(msg)
  {}
  ::traffic_information_msgs::msg::HookStatus is_hooked(::traffic_information_msgs::msg::HookStatus::_is_hooked_type arg)
  {
    msg_.is_hooked = std::move(arg);
    return std::move(msg_);
  }

private:
  ::traffic_information_msgs::msg::HookStatus msg_;
};

class Init_HookStatus_header
{
public:
  Init_HookStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HookStatus_is_hooked header(::traffic_information_msgs::msg::HookStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HookStatus_is_hooked(msg_);
  }

private:
  ::traffic_information_msgs::msg::HookStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::traffic_information_msgs::msg::HookStatus>()
{
  return traffic_information_msgs::msg::builder::Init_HookStatus_header();
}

}  // namespace traffic_information_msgs

#endif  // TRAFFIC_INFORMATION_MSGS__MSG__DETAIL__HOOK_STATUS__BUILDER_HPP_
