// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ur10e_interfaces/srv/detail/validate_trajectory__rosidl_typesupport_introspection_c.h"
#include "ur10e_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ur10e_interfaces/srv/detail/validate_trajectory__functions.h"
#include "ur10e_interfaces/srv/detail/validate_trajectory__struct.h"


// Include directives for member types
// Member `ee_positions_x`
// Member `ee_positions_y`
// Member `ee_positions_z`
// Member `ee_quat`
// Member `sim_time`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ur10e_interfaces__srv__ValidateTrajectory_Request__init(message_memory);
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_fini_function(void * message_memory)
{
  ur10e_interfaces__srv__ValidateTrajectory_Request__fini(message_memory);
}

size_t ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_positions_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_positions_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_positions_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_positions_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_positions_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_positions_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_positions_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_positions_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_positions_z(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_positions_z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_z(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_positions_z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_z(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_positions_z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_quat(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_quat(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_quat(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_quat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_quat(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_quat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_quat(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_quat(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__sim_time(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__sim_time(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__sim_time(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__sim_time(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__sim_time(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__sim_time(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__sim_time(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__sim_time(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_member_array[5] = {
  {
    "ee_positions_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Request, ee_positions_x),  // bytes offset in struct
    NULL,  // default value
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_positions_x,  // size() function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_x,  // get_const(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_x,  // get(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_positions_x,  // fetch(index, &value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_positions_x,  // assign(index, value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_positions_x  // resize(index) function pointer
  },
  {
    "ee_positions_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Request, ee_positions_y),  // bytes offset in struct
    NULL,  // default value
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_positions_y,  // size() function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_y,  // get_const(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_y,  // get(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_positions_y,  // fetch(index, &value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_positions_y,  // assign(index, value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_positions_y  // resize(index) function pointer
  },
  {
    "ee_positions_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Request, ee_positions_z),  // bytes offset in struct
    NULL,  // default value
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_positions_z,  // size() function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_positions_z,  // get_const(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_positions_z,  // get(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_positions_z,  // fetch(index, &value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_positions_z,  // assign(index, value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_positions_z  // resize(index) function pointer
  },
  {
    "ee_quat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Request, ee_quat),  // bytes offset in struct
    NULL,  // default value
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__ee_quat,  // size() function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__ee_quat,  // get_const(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__ee_quat,  // get(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__ee_quat,  // fetch(index, &value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__ee_quat,  // assign(index, value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__ee_quat  // resize(index) function pointer
  },
  {
    "sim_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Request, sim_time),  // bytes offset in struct
    NULL,  // default value
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Request__sim_time,  // size() function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Request__sim_time,  // get_const(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Request__sim_time,  // get(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Request__sim_time,  // fetch(index, &value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Request__sim_time,  // assign(index, value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Request__sim_time  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_members = {
  "ur10e_interfaces__srv",  // message namespace
  "ValidateTrajectory_Request",  // message name
  5,  // number of fields
  sizeof(ur10e_interfaces__srv__ValidateTrajectory_Request),
  ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_member_array,  // message members
  ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_type_support_handle = {
  0,
  &ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur10e_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory_Request)() {
  if (!ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_type_support_handle.typesupport_identifier) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ur10e_interfaces__srv__ValidateTrajectory_Request__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ur10e_interfaces/srv/detail/validate_trajectory__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ur10e_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ur10e_interfaces/srv/detail/validate_trajectory__functions.h"
// already included above
// #include "ur10e_interfaces/srv/detail/validate_trajectory__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `joint_velocities`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ur10e_interfaces__srv__ValidateTrajectory_Response__init(message_memory);
}

void ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_fini_function(void * message_memory)
{
  ur10e_interfaces__srv__ValidateTrajectory_Response__fini(message_memory);
}

size_t ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Response__joint_velocities(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Response__joint_velocities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Response__joint_velocities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Response__joint_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Response__joint_velocities(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Response__joint_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Response__joint_velocities(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Response__joint_velocities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur10e_interfaces__srv__ValidateTrajectory_Response, joint_velocities),  // bytes offset in struct
    NULL,  // default value
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__size_function__ValidateTrajectory_Response__joint_velocities,  // size() function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__get_const_function__ValidateTrajectory_Response__joint_velocities,  // get_const(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__get_function__ValidateTrajectory_Response__joint_velocities,  // get(index) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__fetch_function__ValidateTrajectory_Response__joint_velocities,  // fetch(index, &value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__assign_function__ValidateTrajectory_Response__joint_velocities,  // assign(index, value) function pointer
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__resize_function__ValidateTrajectory_Response__joint_velocities  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_members = {
  "ur10e_interfaces__srv",  // message namespace
  "ValidateTrajectory_Response",  // message name
  3,  // number of fields
  sizeof(ur10e_interfaces__srv__ValidateTrajectory_Response),
  ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_member_array,  // message members
  ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_type_support_handle = {
  0,
  &ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur10e_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory_Response)() {
  if (!ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_type_support_handle.typesupport_identifier) {
    ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ur10e_interfaces__srv__ValidateTrajectory_Response__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ur10e_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ur10e_interfaces/srv/detail/validate_trajectory__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_members = {
  "ur10e_interfaces__srv",  // service namespace
  "ValidateTrajectory",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_Request_message_type_support_handle,
  NULL  // response message
  // ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_Response_message_type_support_handle
};

static rosidl_service_type_support_t ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_type_support_handle = {
  0,
  &ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur10e_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory)() {
  if (!ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_type_support_handle.typesupport_identifier) {
    ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur10e_interfaces, srv, ValidateTrajectory_Response)()->data;
  }

  return &ur10e_interfaces__srv__detail__validate_trajectory__rosidl_typesupport_introspection_c__ValidateTrajectory_service_type_support_handle;
}
