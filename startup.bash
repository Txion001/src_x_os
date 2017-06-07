#!/bin/bash
python -m SimpleHTTPServer 8000 &
echo "Starting Roscore"
source /home/docker/ws/x_os/devel/setup.bash
roscore -p 8001
