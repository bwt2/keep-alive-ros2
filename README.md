## About
ROS 2 snippet, requests estop service if listener doesn't receive any topic updates from talker after a certain timeout. Beta test for keep-alive estop mechanism for Sydney Interplanetary Rover Initiative (SIRI).

## Manual setup
Assuming `ros-humble-desktop` is installed, run:

```bash
cd keep-alive-test
source /opt/ros/humble/setup.bash
colcon build --packages-select keep-alive-estop estop_interfaces estop --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
source install/setup.bash
```
Run each executable:
```bash
ros2 run estop estop_server
ros2 run keep-alive-estop talker_node
ros2 run keep-alive-estop listener_node
```

## Docker
Assuming docker is installed, run the following to launch all executables

```bash
docker compose up
```

To run only one executable, do 

```bash
docker compose run --rm talker
docker compose run --rm listener
docker compose run --rm estop_server
```

To stop any of the containers,

```bash
docker compose stop talker
docker compose stop listener
docker compose stop estop_server
```

To use the docker container as a shell containing all the repositories' (built) code,

```bash
docker build -t keep-alive-test . && docker run -it --rm keep-alive-test
```

## TODO
- test on different devices
- multi-stage builds