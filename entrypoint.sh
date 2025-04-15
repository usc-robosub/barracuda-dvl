source /opt/ros/noetic/setup.bash

catkin_make
source devel/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-dvl/catkin_ws/devel/setup.bash" >> ~/.bashrc

roslaunch barracuda_dvl launch_dvl.launch

