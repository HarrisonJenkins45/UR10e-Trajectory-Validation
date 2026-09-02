# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target ur10e_interfaces::ur10e_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${ur10e_interfaces_TARGETS}.
if(ur10e_interfaces_TARGETS AND NOT TARGET ur10e_interfaces::ur10e_interfaces)
  add_library(ur10e_interfaces::ur10e_interfaces INTERFACE IMPORTED)
  set_target_properties(ur10e_interfaces::ur10e_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ur10e_interfaces_TARGETS}")
endif()
