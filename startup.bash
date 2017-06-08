#!/bin/bash
python -m SimpleHTTPServer 8000 &
echo "Starting Roscore"
source /home/docker/ws/x_os/devel/setup.bash
ROS_PACKAGE_PATH=/home/docker/ws/x_os/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
ROSLISP_PACKAGE_DIRECTORIES=/home/docker/ws/x_os/devel/share/common-lisp:/home/docker/ws/x_os/devel/share/common-lisp
ROS_MASTER_URI=http://127.0.0.1:8001
ROS_IP=127.0.0.10
sleep 5s
roslaunch -p 8001 x_main x_os.launch
