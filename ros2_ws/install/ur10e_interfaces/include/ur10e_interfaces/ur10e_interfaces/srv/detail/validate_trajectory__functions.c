// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice
#include "ur10e_interfaces/srv/detail/validate_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `ee_positions_x`
// Member `ee_positions_y`
// Member `ee_positions_z`
// Member `ee_quat`
// Member `sim_time`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ur10e_interfaces__srv__ValidateTrajectory_Request__init(ur10e_interfaces__srv__ValidateTrajectory_Request * msg)
{
  if (!msg) {
    return false;
  }
  // ee_positions_x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->ee_positions_x, 0)) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__fini(msg);
    return false;
  }
  // ee_positions_y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->ee_positions_y, 0)) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__fini(msg);
    return false;
  }
  // ee_positions_z
  if (!rosidl_runtime_c__double__Sequence__init(&msg->ee_positions_z, 0)) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__fini(msg);
    return false;
  }
  // ee_quat
  if (!rosidl_runtime_c__double__Sequence__init(&msg->ee_quat, 0)) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__fini(msg);
    return false;
  }
  // sim_time
  if (!rosidl_runtime_c__double__Sequence__init(&msg->sim_time, 0)) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__fini(msg);
    return false;
  }
  return true;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Request__fini(ur10e_interfaces__srv__ValidateTrajectory_Request * msg)
{
  if (!msg) {
    return;
  }
  // ee_positions_x
  rosidl_runtime_c__double__Sequence__fini(&msg->ee_positions_x);
  // ee_positions_y
  rosidl_runtime_c__double__Sequence__fini(&msg->ee_positions_y);
  // ee_positions_z
  rosidl_runtime_c__double__Sequence__fini(&msg->ee_positions_z);
  // ee_quat
  rosidl_runtime_c__double__Sequence__fini(&msg->ee_quat);
  // sim_time
  rosidl_runtime_c__double__Sequence__fini(&msg->sim_time);
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Request__are_equal(const ur10e_interfaces__srv__ValidateTrajectory_Request * lhs, const ur10e_interfaces__srv__ValidateTrajectory_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ee_positions_x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->ee_positions_x), &(rhs->ee_positions_x)))
  {
    return false;
  }
  // ee_positions_y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->ee_positions_y), &(rhs->ee_positions_y)))
  {
    return false;
  }
  // ee_positions_z
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->ee_positions_z), &(rhs->ee_positions_z)))
  {
    return false;
  }
  // ee_quat
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->ee_quat), &(rhs->ee_quat)))
  {
    return false;
  }
  // sim_time
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->sim_time), &(rhs->sim_time)))
  {
    return false;
  }
  return true;
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Request__copy(
  const ur10e_interfaces__srv__ValidateTrajectory_Request * input,
  ur10e_interfaces__srv__ValidateTrajectory_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // ee_positions_x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->ee_positions_x), &(output->ee_positions_x)))
  {
    return false;
  }
  // ee_positions_y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->ee_positions_y), &(output->ee_positions_y)))
  {
    return false;
  }
  // ee_positions_z
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->ee_positions_z), &(output->ee_positions_z)))
  {
    return false;
  }
  // ee_quat
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->ee_quat), &(output->ee_quat)))
  {
    return false;
  }
  // sim_time
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->sim_time), &(output->sim_time)))
  {
    return false;
  }
  return true;
}

ur10e_interfaces__srv__ValidateTrajectory_Request *
ur10e_interfaces__srv__ValidateTrajectory_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur10e_interfaces__srv__ValidateTrajectory_Request * msg = (ur10e_interfaces__srv__ValidateTrajectory_Request *)allocator.allocate(sizeof(ur10e_interfaces__srv__ValidateTrajectory_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur10e_interfaces__srv__ValidateTrajectory_Request));
  bool success = ur10e_interfaces__srv__ValidateTrajectory_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Request__destroy(ur10e_interfaces__srv__ValidateTrajectory_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__init(ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur10e_interfaces__srv__ValidateTrajectory_Request * data = NULL;

  if (size) {
    data = (ur10e_interfaces__srv__ValidateTrajectory_Request *)allocator.zero_allocate(size, sizeof(ur10e_interfaces__srv__ValidateTrajectory_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur10e_interfaces__srv__ValidateTrajectory_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur10e_interfaces__srv__ValidateTrajectory_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__fini(ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ur10e_interfaces__srv__ValidateTrajectory_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence *
ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * array = (ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence *)allocator.allocate(sizeof(ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__destroy(ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__are_equal(const ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * lhs, const ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur10e_interfaces__srv__ValidateTrajectory_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__copy(
  const ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * input,
  ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur10e_interfaces__srv__ValidateTrajectory_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur10e_interfaces__srv__ValidateTrajectory_Request * data =
      (ur10e_interfaces__srv__ValidateTrajectory_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur10e_interfaces__srv__ValidateTrajectory_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur10e_interfaces__srv__ValidateTrajectory_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur10e_interfaces__srv__ValidateTrajectory_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `joint_velocities`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ur10e_interfaces__srv__ValidateTrajectory_Response__init(ur10e_interfaces__srv__ValidateTrajectory_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    ur10e_interfaces__srv__ValidateTrajectory_Response__fini(msg);
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_velocities, 0)) {
    ur10e_interfaces__srv__ValidateTrajectory_Response__fini(msg);
    return false;
  }
  return true;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Response__fini(ur10e_interfaces__srv__ValidateTrajectory_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // joint_velocities
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_velocities);
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Response__are_equal(const ur10e_interfaces__srv__ValidateTrajectory_Response * lhs, const ur10e_interfaces__srv__ValidateTrajectory_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_velocities), &(rhs->joint_velocities)))
  {
    return false;
  }
  return true;
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Response__copy(
  const ur10e_interfaces__srv__ValidateTrajectory_Response * input,
  ur10e_interfaces__srv__ValidateTrajectory_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_velocities), &(output->joint_velocities)))
  {
    return false;
  }
  return true;
}

ur10e_interfaces__srv__ValidateTrajectory_Response *
ur10e_interfaces__srv__ValidateTrajectory_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur10e_interfaces__srv__ValidateTrajectory_Response * msg = (ur10e_interfaces__srv__ValidateTrajectory_Response *)allocator.allocate(sizeof(ur10e_interfaces__srv__ValidateTrajectory_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur10e_interfaces__srv__ValidateTrajectory_Response));
  bool success = ur10e_interfaces__srv__ValidateTrajectory_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Response__destroy(ur10e_interfaces__srv__ValidateTrajectory_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur10e_interfaces__srv__ValidateTrajectory_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__init(ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur10e_interfaces__srv__ValidateTrajectory_Response * data = NULL;

  if (size) {
    data = (ur10e_interfaces__srv__ValidateTrajectory_Response *)allocator.zero_allocate(size, sizeof(ur10e_interfaces__srv__ValidateTrajectory_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur10e_interfaces__srv__ValidateTrajectory_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur10e_interfaces__srv__ValidateTrajectory_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__fini(ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ur10e_interfaces__srv__ValidateTrajectory_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence *
ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * array = (ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence *)allocator.allocate(sizeof(ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__destroy(ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__are_equal(const ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * lhs, const ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur10e_interfaces__srv__ValidateTrajectory_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__copy(
  const ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * input,
  ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur10e_interfaces__srv__ValidateTrajectory_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur10e_interfaces__srv__ValidateTrajectory_Response * data =
      (ur10e_interfaces__srv__ValidateTrajectory_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur10e_interfaces__srv__ValidateTrajectory_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur10e_interfaces__srv__ValidateTrajectory_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur10e_interfaces__srv__ValidateTrajectory_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
