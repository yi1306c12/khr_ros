#https://hub.docker.com/r/_/ros/

FROM ros:kinetic

RUN apt update -y && apt upgrade -y
RUN apt install -y git
RUN apt install -y ros-kinetic-ros-tutorials ros-kinetic-common-tutorials

# add user
RUN useradd -ms /bin/bash rosuser
USER rosuser

# init workspace
WORKDIR /home/rosuser
RUN mkdir catkin_ws/src -p
WORKDIR catkin_ws/src
RUN bash /opt/ros/kinetic/setup.bash catkin_init_workspace

# download & compile sources
RUN git clone https://github.com/yi1306c12/khr_ros
WORKDIR /home/rosuser/catkin_ws
RUN catkin_make

# set home directory
WORKDIR /home/rosuser
