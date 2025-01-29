cmake_minimum_required(VERSION 3.8)
project(estop_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate service interfacexs
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Estop.srv"
)

ament_package()