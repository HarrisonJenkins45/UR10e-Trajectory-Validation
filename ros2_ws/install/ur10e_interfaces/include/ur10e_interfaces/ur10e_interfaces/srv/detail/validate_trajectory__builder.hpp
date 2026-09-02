// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice

#ifndef UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__BUILDER_HPP_
#define UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur10e_interfaces/srv/detail/validate_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur10e_interfaces
{

namespace srv
{

namespace builder
{

class Init_ValidateTrajectory_Request_sim_time
{
public:
  explicit Init_ValidateTrajectory_Request_sim_time(::ur10e_interfaces::srv::ValidateTrajectory_Request & msg)
  : msg_(msg)
  {}
  ::ur10e_interfaces::srv::ValidateTrajectory_Request sim_time(::ur10e_interfaces::srv::ValidateTrajectory_Request::_sim_time_type arg)
  {
    msg_.sim_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Request msg_;
};

class Init_ValidateTrajectory_Request_ee_quat
{
public:
  explicit Init_ValidateTrajectory_Request_ee_quat(::ur10e_interfaces::srv::ValidateTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_ValidateTrajectory_Request_sim_time ee_quat(::ur10e_interfaces::srv::ValidateTrajectory_Request::_ee_quat_type arg)
  {
    msg_.ee_quat = std::move(arg);
    return Init_ValidateTrajectory_Request_sim_time(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Request msg_;
};

class Init_ValidateTrajectory_Request_ee_positions_z
{
public:
  explicit Init_ValidateTrajectory_Request_ee_positions_z(::ur10e_interfaces::srv::ValidateTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_ValidateTrajectory_Request_ee_quat ee_positions_z(::ur10e_interfaces::srv::ValidateTrajectory_Request::_ee_positions_z_type arg)
  {
    msg_.ee_positions_z = std::move(arg);
    return Init_ValidateTrajectory_Request_ee_quat(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Request msg_;
};

class Init_ValidateTrajectory_Request_ee_positions_y
{
public:
  explicit Init_ValidateTrajectory_Request_ee_positions_y(::ur10e_interfaces::srv::ValidateTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_ValidateTrajectory_Request_ee_positions_z ee_positions_y(::ur10e_interfaces::srv::ValidateTrajectory_Request::_ee_positions_y_type arg)
  {
    msg_.ee_positions_y = std::move(arg);
    return Init_ValidateTrajectory_Request_ee_positions_z(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Request msg_;
};

class Init_ValidateTrajectory_Request_ee_positions_x
{
public:
  Init_ValidateTrajectory_Request_ee_positions_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ValidateTrajectory_Request_ee_positions_y ee_positions_x(::ur10e_interfaces::srv::ValidateTrajectory_Request::_ee_positions_x_type arg)
  {
    msg_.ee_positions_x = std::move(arg);
    return Init_ValidateTrajectory_Request_ee_positions_y(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur10e_interfaces::srv::ValidateTrajectory_Request>()
{
  return ur10e_interfaces::srv::builder::Init_ValidateTrajectory_Request_ee_positions_x();
}

}  // namespace ur10e_interfaces


namespace ur10e_interfaces
{

namespace srv
{

namespace builder
{

class Init_ValidateTrajectory_Response_joint_velocities
{
public:
  explicit Init_ValidateTrajectory_Response_joint_velocities(::ur10e_interfaces::srv::ValidateTrajectory_Response & msg)
  : msg_(msg)
  {}
  ::ur10e_interfaces::srv::ValidateTrajectory_Response joint_velocities(::ur10e_interfaces::srv::ValidateTrajectory_Response::_joint_velocities_type arg)
  {
    msg_.joint_velocities = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Response msg_;
};

class Init_ValidateTrajectory_Response_message
{
public:
  explicit Init_ValidateTrajectory_Response_message(::ur10e_interfaces::srv::ValidateTrajectory_Response & msg)
  : msg_(msg)
  {}
  Init_ValidateTrajectory_Response_joint_velocities message(::ur10e_interfaces::srv::ValidateTrajectory_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ValidateTrajectory_Response_joint_velocities(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Response msg_;
};

class Init_ValidateTrajectory_Response_success
{
public:
  Init_ValidateTrajectory_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ValidateTrajectory_Response_message success(::ur10e_interfaces::srv::ValidateTrajectory_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ValidateTrajectory_Response_message(msg_);
  }

private:
  ::ur10e_interfaces::srv::ValidateTrajectory_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur10e_interfaces::srv::ValidateTrajectory_Response>()
{
  return ur10e_interfaces::srv::builder::Init_ValidateTrajectory_Response_success();
}

}  // namespace ur10e_interfaces

#endif  // UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__BUILDER_HPP_
