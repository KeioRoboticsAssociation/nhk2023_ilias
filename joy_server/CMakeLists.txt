cmake_minimum_required(VERSION 3.8)
project(joy_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rogilink2_interfaces REQUIRED)

add_executable(joy_server src/joy_server.cpp)
ament_target_dependencies(joy_server
  rclcpp
  sensor_msgs
  std_msgs
  rogilink2_interfaces
)

install(TARGETS
  joy_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()