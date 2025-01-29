#!bin/bash

colcon build --packages-select keep-alive-estop estop_interfaces estop --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 

# ros2 run keep-alive-estop talker
# ros2 run keep-alive-estop lsitener

# ros2 run estop estop_server
# ros2 service call /estop_server estop_interfaces/srv/Estop "{start_estop: false}"