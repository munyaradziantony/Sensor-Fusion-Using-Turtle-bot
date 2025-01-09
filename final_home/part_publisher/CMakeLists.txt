cmake_minimum_required(VERSION 3.8)
project(part_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)

include_directories(include/part_publisher)

add_executable(part_publisher_exe src/main.cpp src/part_publisher_node.cpp)

ament_target_dependencies(part_publisher_exe rclcpp mage_msgs)

install(TARGETS part_publisher_exe DESTINATION lib/${PROJECT_NAME})


ament_package()
