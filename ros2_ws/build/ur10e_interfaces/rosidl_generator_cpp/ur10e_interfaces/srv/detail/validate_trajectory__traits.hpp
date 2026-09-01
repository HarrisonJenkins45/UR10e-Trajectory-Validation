// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ur10e_interfaces:srv/ValidateTrajectory.idl
// generated code does not contain a copyright notice

#ifndef UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__TRAITS_HPP_
#define UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ur10e_interfaces/srv/detail/validate_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ur10e_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ValidateTrajectory_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: ee_positions_x
  {
    if (msg.ee_positions_x.size() == 0) {
      out << "ee_positions_x: []";
    } else {
      out << "ee_positions_x: [";
      size_t pending_items = msg.ee_positions_x.size();
      for (auto item : msg.ee_positions_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_positions_y
  {
    if (msg.ee_positions_y.size() == 0) {
      out << "ee_positions_y: []";
    } else {
      out << "ee_positions_y: [";
      size_t pending_items = msg.ee_positions_y.size();
      for (auto item : msg.ee_positions_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_positions_z
  {
    if (msg.ee_positions_z.size() == 0) {
      out << "ee_positions_z: []";
    } else {
      out << "ee_positions_z: [";
      size_t pending_items = msg.ee_positions_z.size();
      for (auto item : msg.ee_positions_z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_quat
  {
    if (msg.ee_quat.size() == 0) {
      out << "ee_quat: []";
    } else {
      out << "ee_quat: [";
      size_t pending_items = msg.ee_quat.size();
      for (auto item : msg.ee_quat) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: sim_time
  {
    if (msg.sim_time.size() == 0) {
      out << "sim_time: []";
    } else {
      out << "sim_time: [";
      size_t pending_items = msg.sim_time.size();
      for (auto item : msg.sim_time) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ValidateTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ee_positions_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_positions_x.size() == 0) {
      out << "ee_positions_x: []\n";
    } else {
      out << "ee_positions_x:\n";
      for (auto item : msg.ee_positions_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_positions_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_positions_y.size() == 0) {
      out << "ee_positions_y: []\n";
    } else {
      out << "ee_positions_y:\n";
      for (auto item : msg.ee_positions_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_positions_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_positions_z.size() == 0) {
      out << "ee_positions_z: []\n";
    } else {
      out << "ee_positions_z:\n";
      for (auto item : msg.ee_positions_z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_quat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_quat.size() == 0) {
      out << "ee_quat: []\n";
    } else {
      out << "ee_quat:\n";
      for (auto item : msg.ee_quat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: sim_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sim_time.size() == 0) {
      out << "sim_time: []\n";
    } else {
      out << "sim_time:\n";
      for (auto item : msg.sim_time) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ValidateTrajectory_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ur10e_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ur10e_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur10e_interfaces::srv::ValidateTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur10e_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur10e_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur10e_interfaces::srv::ValidateTrajectory_Request & msg)
{
  return ur10e_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur10e_interfaces::srv::ValidateTrajectory_Request>()
{
  return "ur10e_interfaces::srv::ValidateTrajectory_Request";
}

template<>
inline const char * name<ur10e_interfaces::srv::ValidateTrajectory_Request>()
{
  return "ur10e_interfaces/srv/ValidateTrajectory_Request";
}

template<>
struct has_fixed_size<ur10e_interfaces::srv::ValidateTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ur10e_interfaces::srv::ValidateTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ur10e_interfaces::srv::ValidateTrajectory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ur10e_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ValidateTrajectory_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: joint_velocities
  {
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []";
    } else {
      out << "joint_velocities: [";
      size_t pending_items = msg.joint_velocities.size();
      for (auto item : msg.joint_velocities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ValidateTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: joint_velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []\n";
    } else {
      out << "joint_velocities:\n";
      for (auto item : msg.joint_velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ValidateTrajectory_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ur10e_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ur10e_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur10e_interfaces::srv::ValidateTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur10e_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur10e_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur10e_interfaces::srv::ValidateTrajectory_Response & msg)
{
  return ur10e_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur10e_interfaces::srv::ValidateTrajectory_Response>()
{
  return "ur10e_interfaces::srv::ValidateTrajectory_Response";
}

template<>
inline const char * name<ur10e_interfaces::srv::ValidateTrajectory_Response>()
{
  return "ur10e_interfaces/srv/ValidateTrajectory_Response";
}

template<>
struct has_fixed_size<ur10e_interfaces::srv::ValidateTrajectory_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ur10e_interfaces::srv::ValidateTrajectory_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ur10e_interfaces::srv::ValidateTrajectory_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ur10e_interfaces::srv::ValidateTrajectory>()
{
  return "ur10e_interfaces::srv::ValidateTrajectory";
}

template<>
inline const char * name<ur10e_interfaces::srv::ValidateTrajectory>()
{
  return "ur10e_interfaces/srv/ValidateTrajectory";
}

template<>
struct has_fixed_size<ur10e_interfaces::srv::ValidateTrajectory>
  : std::integral_constant<
    bool,
    has_fixed_size<ur10e_interfaces::srv::ValidateTrajectory_Request>::value &&
    has_fixed_size<ur10e_interfaces::srv::ValidateTrajectory_Response>::value
  >
{
};

template<>
struct has_bounded_size<ur10e_interfaces::srv::ValidateTrajectory>
  : std::integral_constant<
    bool,
    has_bounded_size<ur10e_interfaces::srv::ValidateTrajectory_Request>::value &&
    has_bounded_size<ur10e_interfaces::srv::ValidateTrajectory_Response>::value
  >
{
};

template<>
struct is_service<ur10e_interfaces::srv::ValidateTrajectory>
  : std::true_type
{
};

template<>
struct is_service_request<ur10e_interfaces::srv::ValidateTrajectory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ur10e_interfaces::srv::ValidateTrajectory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UR10E_INTERFACES__SRV__DETAIL__VALIDATE_TRAJECTORY__TRAITS_HPP_
