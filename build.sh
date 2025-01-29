#!bin/bash

source /opt/ros/humble/setup.bash
colcon build --packages-select keep-alive-estop --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
source install/setup.bash

# ros2 run keep-alive-test listener_node
# ros2 run keep-alive-test talker_node