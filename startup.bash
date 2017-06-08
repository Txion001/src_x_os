#!/bin/bash
python -m SimpleHTTPServer 8000 &
echo "Starting Roscore"
ROS_PACKAGE_PATH=/home/docker/ws/x_os/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
ROS_MASTER_URI=http://127.0.0.1:8001
source /home/docker/ws/x_os/devel/setup.bash
roslaunch x_main x_os.launch
