#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

cd /opt/barracuda-dvl/catkin_ws
colcon build --symlink-install
source install/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-dvl/catkin_ws/install/setup.bash" >> ~/.bashrc

# ros2 launch waterlinked_dvl launch_dvl.launch.py
ros2 run waterlinked_dvl barracuda-dvl-ros-driver.py --ros-args -p dvl_host:='192.168.8.148' -p client_address:='0.0.0.0'
# ros2 run dvl_a50 dvl_a50.py --ros-args -p dvl_address:='192.168.194.95' -p client_address:='0.0.0.0'
# ros2 launch dvl_a50 dvl_a50.launch.py ip_address:='192.168.194.95'
