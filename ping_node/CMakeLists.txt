cmake_minimum_required(VERSION 3.8)
project(ping_node)

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
find_package(rogilink2_interfaces REQUIRED)


add_executable(ping_node src/main.cpp)
ament_target_dependencies(ping_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  sensor_lib
)

install(TARGETS
  ping_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()