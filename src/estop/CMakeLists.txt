cmake_minimum_required(VERSION 3.8)
project(estop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(estop_interfaces REQUIRED)

# declare executables alongside dependencies
add_executable(estop_server src/estop_server.cpp)
ament_target_dependencies(estop_server rclcpp estop_interfaces)

# add estop/include directory into include path when building estop_server executable
target_include_directories(estop_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# let ros2 run find the executable
install(TARGETS
  estop_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
