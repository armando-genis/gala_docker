// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/HookStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__HOOK_STATUS__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__HOOK_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__HookStatus __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__HookStatus __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HookStatus_
{
  using Type = HookStatus_<ContainerAllocator>;

  explicit HookStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_hooked = false;
    }
  }

  explicit HookStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_hooked = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_hooked_type =
    bool;
  _is_hooked_type is_hooked;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_hooked(
    const bool & _arg)
  {
    this->is_hooked = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::HookStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::HookStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::HookStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::HookStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__HookStatus
    std::shared_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__HookStatus
    std::shared_ptr<custom_interfaces::msg::HookStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HookStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_hooked != other.is_hooked) {
      return false;
    }
    return true;
  }
  bool operator!=(const HookStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HookStatus_

// alias to use template instance with default allocator
using HookStatus =
  custom_interfaces::msg::HookStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__HOOK_STATUS__STRUCT_HPP_
