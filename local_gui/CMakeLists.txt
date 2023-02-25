cmake_minimum_required(VERSION 3.8)
project(local_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/local_gui
  DESTINATION lib/${PROJECT_NAME})

ament_package()
