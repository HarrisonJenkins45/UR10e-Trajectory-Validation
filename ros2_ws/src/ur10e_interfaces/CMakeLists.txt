cmake_minimum_required(VERSION 3.8)
project(ur10e_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Find the rosidl generator package
find_package(rosidl_default_generators REQUIRED)

# Declare your service files relative to the package root
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ValidateTrajectory.srv"
)

ament_package()
