cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mage_msgs REQUIRED)


# Include directories
include_directories(
  include/${PROJECT_NAME}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Define executables
add_executable(waypoint_main
  src/main.cpp
  src/waypoint_publisher.cpp
  src/waypoint_reacher.cpp
  src/color_utils.cpp
  src/part_pose_finder.cpp
)

# Dependencies for executables
ament_target_dependencies(
  waypoint_main
  rclcpp nav_msgs tf2_geometry_msgs std_msgs rcutils mage_msgs
)

# Install
install(DIRECTORY include/
  DESTINATION include)

install(TARGETS waypoint_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files (if any)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch*"
)

ament_package()