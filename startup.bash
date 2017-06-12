#!/bin/bash
python -m SimpleHTTPServer 8000 &
echo "Starting Roscore"
source /home/docker/ws/x_os/devel/setup.bash
ROS_PACKAGE_PATH=/home/docker/ws/x_os/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
ROSLISP_PACKAGE_DIRECTORIES=/home/docker/ws/x_os/devel/share/common-lisp:/home/docker/ws/x_os/devel/share/common-lisp
ROS_MASTER_URI=http://192.168.2.10:11311
ROS_IP=192.168.2.10
sleep 5s
roslaunch x_main x_os.launch
