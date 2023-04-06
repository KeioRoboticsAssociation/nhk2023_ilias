cmake_minimum_required(VERSION 3.8)
project(er_state_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# ament_auto_add_library(state_ctrl SHARED
# src/main.cpp
# src/idle.cpp
# src/manual.cpp
# src/start.cpp
# src/main.hpp
# src/state_ctrl.hpp
# include/tinyfsm/include/tinyfsm.hpp
# )
add_executable(er_state_ctrl
  src/main.cpp
  src/idle.cpp
  src/manual.cpp
  src/start.cpp
  src/pickupLeft.cpp
  src/pickupRight.cpp
  src/shot.cpp
  src/preshot.cpp
  src/robot_state_ctrl.cpp
  src/state_ctrl.cpp
)

ament_target_dependencies(er_state_ctrl
  rclcpp
  std_msgs
)

# ament_target_dependencies(md_lib rclcpp std_msgs rogilink2_interfaces)
# target_include_directories(
# md_lib
# PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# )
# install(
# TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(
# DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  er_state_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()