// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ur10e_interfaces/srv/detail/validate_trajectory__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ur10e_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ValidateTrajectory_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ur10e_interfaces::srv::ValidateTrajectory_Request(_init);
}

void ValidateTrajectory_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ur10e_interfaces::srv::ValidateTrajectory_Request *>(message_memory);
  typed_message->~ValidateTrajectory_Request();
}

size_t size_function__ValidateTrajectory_Request__ee_positions_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ValidateTrajectory_Request__ee_positions_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__ValidateTrajectory_Request__ee_positions_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__ValidateTrajectory_Request__ee_positions_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ValidateTrajectory_Request__ee_positions_x(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ValidateTrajectory_Request__ee_positions_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ValidateTrajectory_Request__ee_positions_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__ValidateTrajectory_Request__ee_positions_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ValidateTrajectory_Request__ee_positions_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ValidateTrajectory_Request__ee_positions_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__ValidateTrajectory_Request__ee_positions_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__ValidateTrajectory_Request__ee_positions_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ValidateTrajectory_Request__ee_positions_y(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ValidateTrajectory_Request__ee_positions_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ValidateTrajectory_Request__ee_positions_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__ValidateTrajectory_Request__ee_positions_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ValidateTrajectory_Request__ee_positions_z(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ValidateTrajectory_Request__ee_positions_z(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__ValidateTrajectory_Request__ee_positions_z(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__ValidateTrajectory_Request__ee_positions_z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ValidateTrajectory_Request__ee_positions_z(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ValidateTrajectory_Request__ee_positions_z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ValidateTrajectory_Request__ee_positions_z(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__ValidateTrajectory_Request__ee_positions_z(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ValidateTrajectory_Request__ee_quat(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ValidateTrajectory_Request__ee_quat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__ValidateTrajectory_Request__ee_quat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__ValidateTrajectory_Request__ee_quat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ValidateTrajectory_Request__ee_quat(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ValidateTrajectory_Request__ee_quat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ValidateTrajectory_Request__ee_quat(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__ValidateTrajectory_Request__ee_quat(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ValidateTrajectory_Request__sim_time(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ValidateTrajectory_Request__sim_time(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__ValidateTrajectory_Request__sim_time(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__ValidateTrajectory_Request__sim_time(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ValidateTrajectory_Request__sim_time(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ValidateTrajectory_Request__sim_time(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ValidateTrajectory_Request__sim_time(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__ValidateTrajectory_Request__sim_time(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ValidateTrajectory_Request_message_member_array[5] = {
  {
    "ee_positions_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Request, ee_positions_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__ValidateTrajectory_Request__ee_positions_x,  // size() function pointer
    get_const_function__ValidateTrajectory_Request__ee_positions_x,  // get_const(index) function pointer
    get_function__ValidateTrajectory_Request__ee_positions_x,  // get(index) function pointer
    fetch_function__ValidateTrajectory_Request__ee_positions_x,  // fetch(index, &value) function pointer
    assign_function__ValidateTrajectory_Request__ee_positions_x,  // assign(index, value) function pointer
    resize_function__ValidateTrajectory_Request__ee_positions_x  // resize(index) function pointer
  },
  {
    "ee_positions_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Request, ee_positions_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__ValidateTrajectory_Request__ee_positions_y,  // size() function pointer
    get_const_function__ValidateTrajectory_Request__ee_positions_y,  // get_const(index) function pointer
    get_function__ValidateTrajectory_Request__ee_positions_y,  // get(index) function pointer
    fetch_function__ValidateTrajectory_Request__ee_positions_y,  // fetch(index, &value) function pointer
    assign_function__ValidateTrajectory_Request__ee_positions_y,  // assign(index, value) function pointer
    resize_function__ValidateTrajectory_Request__ee_positions_y  // resize(index) function pointer
  },
  {
    "ee_positions_z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Request, ee_positions_z),  // bytes offset in struct
    nullptr,  // default value
    size_function__ValidateTrajectory_Request__ee_positions_z,  // size() function pointer
    get_const_function__ValidateTrajectory_Request__ee_positions_z,  // get_const(index) function pointer
    get_function__ValidateTrajectory_Request__ee_positions_z,  // get(index) function pointer
    fetch_function__ValidateTrajectory_Request__ee_positions_z,  // fetch(index, &value) function pointer
    assign_function__ValidateTrajectory_Request__ee_positions_z,  // assign(index, value) function pointer
    resize_function__ValidateTrajectory_Request__ee_positions_z  // resize(index) function pointer
  },
  {
    "ee_quat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Request, ee_quat),  // bytes offset in struct
    nullptr,  // default value
    size_function__ValidateTrajectory_Request__ee_quat,  // size() function pointer
    get_const_function__ValidateTrajectory_Request__ee_quat,  // get_const(index) function pointer
    get_function__ValidateTrajectory_Request__ee_quat,  // get(index) function pointer
    fetch_function__ValidateTrajectory_Request__ee_quat,  // fetch(index, &value) function pointer
    assign_function__ValidateTrajectory_Request__ee_quat,  // assign(index, value) function pointer
    resize_function__ValidateTrajectory_Request__ee_quat  // resize(index) function pointer
  },
  {
    "sim_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Request, sim_time),  // bytes offset in struct
    nullptr,  // default value
    size_function__ValidateTrajectory_Request__sim_time,  // size() function pointer
    get_const_function__ValidateTrajectory_Request__sim_time,  // get_const(index) function pointer
    get_function__ValidateTrajectory_Request__sim_time,  // get(index) function pointer
    fetch_function__ValidateTrajectory_Request__sim_time,  // fetch(index, &value) function pointer
    assign_function__ValidateTrajectory_Request__sim_time,  // assign(index, value) function pointer
    resize_function__ValidateTrajectory_Request__sim_time  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ValidateTrajectory_Request_message_members = {
  "ur10e_interfaces::srv",  // message namespace
  "ValidateTrajectory_Request",  // message name
  5,  // number of fields
  sizeof(ur10e_interfaces::srv::ValidateTrajectory_Request),
  ValidateTrajectory_Request_message_member_array,  // message members
  ValidateTrajectory_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ValidateTrajectory_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ValidateTrajectory_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ValidateTrajectory_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ur10e_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ur10e_interfaces::srv::ValidateTrajectory_Request>()
{
  return &::ur10e_interfaces::srv::rosidl_typesupport_introspection_cpp::ValidateTrajectory_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ur10e_interfaces, srv, ValidateTrajectory_Request)() {
  return &::ur10e_interfaces::srv::rosidl_typesupport_introspection_cpp::ValidateTrajectory_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ur10e_interfaces/srv/detail/validate_trajectory__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ur10e_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ValidateTrajectory_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ur10e_interfaces::srv::ValidateTrajectory_Response(_init);
}

void ValidateTrajectory_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ur10e_interfaces::srv::ValidateTrajectory_Response *>(message_memory);
  typed_message->~ValidateTrajectory_Response();
}

size_t size_function__ValidateTrajectory_Response__joint_velocities(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ValidateTrajectory_Response__joint_velocities(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__ValidateTrajectory_Response__joint_velocities(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__ValidateTrajectory_Response__joint_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ValidateTrajectory_Response__joint_velocities(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ValidateTrajectory_Response__joint_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ValidateTrajectory_Response__joint_velocities(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__ValidateTrajectory_Response__joint_velocities(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ValidateTrajectory_Response_message_member_array[3] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_velocities",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces::srv::ValidateTrajectory_Response, joint_velocities),  // bytes offset in struct
    nullptr,  // default value
    size_function__ValidateTrajectory_Response__joint_velocities,  // size() function pointer
    get_const_function__ValidateTrajectory_Response__joint_velocities,  // get_const(index) function pointer
    get_function__ValidateTrajectory_Response__joint_velocities,  // get(index) function pointer
    fetch_function__ValidateTrajectory_Response__joint_velocities,  // fetch(index, &value) function pointer
    assign_function__ValidateTrajectory_Response__joint_velocities,  // assign(index, value) function pointer
    resize_function__ValidateTrajectory_Response__joint_velocities  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ValidateTrajectory_Response_message_members = {
  "ur10e_interfaces::srv",  // message namespace
  "ValidateTrajectory_Response",  // message name
  3,  // number of fields
  sizeof(ur10e_interfaces::srv::ValidateTrajectory_Response),
  ValidateTrajectory_Response_message_member_array,  // message members
  ValidateTrajectory_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ValidateTrajectory_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ValidateTrajectory_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ValidateTrajectory_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ur10e_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ur10e_interfaces::srv::ValidateTrajectory_Response>()
{
  return &::ur10e_interfaces::srv::rosidl_typesupport_introspection_cpp::ValidateTrajectory_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ur10e_interfaces, srv, ValidateTrajectory_Response)() {
  return &::ur10e_interfaces::srv::rosidl_typesupport_introspection_cpp::ValidateTrajectory_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "ur10e_interfaces/srv/detail/validate_trajectory__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace ur10e_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers ValidateTrajectory_service_members = {
  "ur10e_interfaces::srv",  // service namespace
  "ValidateTrajectory",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<ur10e_interfaces::srv::ValidateTrajectory>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t ValidateTrajectory_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ValidateTrajectory_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ur10e_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<ur10e_interfaces::srv::ValidateTrajectory>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::ur10e_interfaces::srv::rosidl_typesupport_introspection_cpp::ValidateTrajectory_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ur10e_interfaces::srv::ValidateTrajectory_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ur10e_interfaces::srv::ValidateTrajectory_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ur10e_interfaces, srv, ValidateTrajectory)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<ur10e_interfaces::srv::ValidateTrajectory>();
}

#ifdef __cplusplus
}
#endif
