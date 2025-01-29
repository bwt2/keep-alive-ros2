cmake_minimum_required(VERSION 3.8)
project(keep-alive-estop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# declare executables alongside dependencies
add_executable(talker_node  src/talker.cpp)
ament_target_dependencies(talker_node  rclcpp std_msgs)

add_executable(listener_node src/listener.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)

install(TARGETS
  talker_node
  listener_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
