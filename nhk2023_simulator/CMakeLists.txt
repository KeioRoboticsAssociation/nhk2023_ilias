cmake_minimum_required(VERSION 3.8)
project(nhk2023_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_tf_publisher src/odom_tf_publisher.cpp)
ament_target_dependencies(odom_tf_publisher
  rclcpp
  nav_msgs
  tf2_ros
)


install(TARGETS
  odom_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()