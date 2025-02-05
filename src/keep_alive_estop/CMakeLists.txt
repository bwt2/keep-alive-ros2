cmake_minimum_required(VERSION 3.8)
project(keep_alive_estop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(estop_interfaces REQUIRED)

# declare executables alongside dependencies
add_executable(keep_alive_estop_talker_node  src/keep_alive_estop_talker.cpp)
ament_target_dependencies(keep_alive_estop_talker_node rclcpp std_msgs)

add_executable(keep_alive_estop_listener_node src/keep_alive_estop_listener.cpp)
ament_target_dependencies(keep_alive_estop_listener_node rclcpp std_msgs estop_interfaces)

# add include directories for keep_alive_estop_talker_node/listener builds
target_include_directories(keep_alive_estop_talker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(keep_alive_estop_listener_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  keep_alive_estop_talker_node
  keep_alive_estop_listener_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
