cmake_minimum_required(VERSION 3.8)
project(robot_ctrl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_lib REQUIRED)
find_package(pure_pursuit_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(rr_robot_ctrl src/rr/robot_ctrl.cpp)
ament_target_dependencies(rr_robot_ctrl
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  sensor_lib
  pure_pursuit_interface
  tf2_ros
  tf2_geometry_msgs
  tf2
)
add_executable(er_robot_ctrl src/er/robot_ctrl.cpp)
ament_target_dependencies(er_robot_ctrl
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  sensor_lib
  pure_pursuit_interface
  tf2_ros
  tf2_geometry_msgs
  tf2
)

install(TARGETS
  rr_robot_ctrl
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  er_robot_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()