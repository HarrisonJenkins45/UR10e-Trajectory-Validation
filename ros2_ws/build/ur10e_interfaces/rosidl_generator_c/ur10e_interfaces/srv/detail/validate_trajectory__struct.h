// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice

#ifndef UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__STRUCT_H_
#define UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ee_positions_x'
// Member 'ee_positions_y'
// Member 'ee_positions_z'
// Member 'ee_quat'
// Member 'sim_time'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ValidateTrajectory in the package ur10e_interfaces.
typedef struct ur10e_interfaces__srv__ValidateTrajectory_Request
{
  /// 1. REQUEST (Input sent to the service)
  /// Desired EE trajectory path (flattened matrix or list of poses)
  rosidl_runtime_c__double__Sequence ee_positions_x;
  rosidl_runtime_c__double__Sequence ee_positions_y;
  rosidl_runtime_c__double__Sequence ee_positions_z;
  rosidl_runtime_c__double__Sequence ee_quat;
  rosidl_runtime_c__double__Sequence sim_time;
} ur10e_interfaces__srv__ValidateTrajectory_Request;

// Struct for a sequence of ur10e_interfaces__srv__ValidateTrajectory_Request.
typedef struct ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence
{
  ur10e_interfaces__srv__ValidateTrajectory_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'joint_velocities'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ValidateTrajectory in the package ur10e_interfaces.
typedef struct ur10e_interfaces__srv__ValidateTrajectory_Response
{
  bool success;
  rosidl_runtime_c__String message;
  /// Approved velocity commands (flattened)
  rosidl_runtime_c__double__Sequence joint_velocities;
} ur10e_interfaces__srv__ValidateTrajectory_Response;

// Struct for a sequence of ur10e_interfaces__srv__ValidateTrajectory_Response.
typedef struct ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence
{
  ur10e_interfaces__srv__ValidateTrajectory_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__STRUCT_H_
