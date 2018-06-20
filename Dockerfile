#https://hub.docker.com/r/_/ros/

FROM ros:kinetic

RUN apt update -y && apt upgrade -y
RUN apt install -y ros-kinetic-ros-tutorials ros-kinetic-common-tutorials

# init workspace
RUN mkdir /catkin_ws/src
WORKDIR /catkin_ws
RUN source /opt/ros/kinetic/setup.bash
RUN catkin_make