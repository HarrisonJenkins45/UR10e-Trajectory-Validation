// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice

#ifndef UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__STRUCT_HPP_
#define UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Request __attribute__((deprecated))
#else
# define DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Request __declspec(deprecated)
#endif

namespace ur10e_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ValidateTrajectory_Request_
{
  using Type = ValidateTrajectory_Request_<ContainerAllocator>;

  explicit ValidateTrajectory_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ValidateTrajectory_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _ee_positions_x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _ee_positions_x_type ee_positions_x;
  using _ee_positions_y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _ee_positions_y_type ee_positions_y;
  using _ee_positions_z_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _ee_positions_z_type ee_positions_z;
  using _ee_quat_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _ee_quat_type ee_quat;
  using _sim_time_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _sim_time_type sim_time;

  // setters for named parameter idiom
  Type & set__ee_positions_x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->ee_positions_x = _arg;
    return *this;
  }
  Type & set__ee_positions_y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->ee_positions_y = _arg;
    return *this;
  }
  Type & set__ee_positions_z(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->ee_positions_z = _arg;
    return *this;
  }
  Type & set__ee_quat(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->ee_quat = _arg;
    return *this;
  }
  Type & set__sim_time(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->sim_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Request
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Request
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ValidateTrajectory_Request_ & other) const
  {
    if (this->ee_positions_x != other.ee_positions_x) {
      return false;
    }
    if (this->ee_positions_y != other.ee_positions_y) {
      return false;
    }
    if (this->ee_positions_z != other.ee_positions_z) {
      return false;
    }
    if (this->ee_quat != other.ee_quat) {
      return false;
    }
    if (this->sim_time != other.sim_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const ValidateTrajectory_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ValidateTrajectory_Request_

// alias to use template instance with default allocator
using ValidateTrajectory_Request =
  ur10e_interfaces::srv::ValidateTrajectory_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ur10e_interfaces


#ifndef _WIN32
# define DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Response __attribute__((deprecated))
#else
# define DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Response __declspec(deprecated)
#endif

namespace ur10e_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ValidateTrajectory_Response_
{
  using Type = ValidateTrajectory_Response_<ContainerAllocator>;

  explicit ValidateTrajectory_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ValidateTrajectory_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _joint_velocities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_velocities_type joint_velocities;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__joint_velocities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_velocities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Response
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur10e_interfaces__srv__ValidateTrajectory_Response
    std::shared_ptr<ur10e_interfaces::srv::ValidateTrajectory_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ValidateTrajectory_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->joint_velocities != other.joint_velocities) {
      return false;
    }
    return true;
  }
  bool operator!=(const ValidateTrajectory_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ValidateTrajectory_Response_

// alias to use template instance with default allocator
using ValidateTrajectory_Response =
  ur10e_interfaces::srv::ValidateTrajectory_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ur10e_interfaces

namespace ur10e_interfaces
{

namespace srv
{

struct ValidateTrajectory
{
  using Request = ur10e_interfaces::srv::ValidateTrajectory_Request;
  using Response = ur10e_interfaces::srv::ValidateTrajectory_Response;
};

}  // namespace srv

}  // namespace ur10e_interfaces

#endif  // UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__STRUCT_HPP_
