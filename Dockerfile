FROM ros:indigo-ros-base

# add ihmc messages
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    python-catkin-tools \
    ros-indigo-catkin \
    ros-indigo-ihmc-msgs \
    ros-indigo-rosbag \
    ros-indigo-tf \
    ros-indigo-tf2 \
    ros-indigo-pcl-ros \
    ros-indigo-pcl-conversions \
    ros-indigo-pcl-msgs \
 && rm -rf /var/lib/apt/lists/*

# clone srcsim
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim

# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
 && catkin build

EXPOSE 8000
EXPOSE 11311
ENV ROS_MASTER_URI http://127.0.0.1:11311

# Copy contents of workspace
ADD x_os x_os

# startup script
# simple HTTP server and a roscore
ADD startup.bash startup.bash
CMD ["./startup.bash"]
