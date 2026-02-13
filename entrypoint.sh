#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

cd /opt/barracuda-dvl/catkin_ws
colcon build --symlink-install
source install/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-dvl/catkin_ws/install/setup.bash" >> ~/.bashrc

ros2 launch waterlinked_dvl launch_dvl.launch.py

