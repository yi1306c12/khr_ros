#https://hub.docker.com/r/_/ros/

FROM ros:melodic
#for debug
#FROM ros_kinetic_updated


RUN apt update -y && apt upgrade -y
RUN apt install -y git

# for running tutorials
#RUN apt install -y ros-kinetic-ros-tutorials ros-kinetic-common-tutorials

# add devel.setup
#RUN sed -e "$ a #!/bin/bash"
#RUN sed -e "$ i [[ -f  /home/rosuser/catkin_ws/devel/setup.bash ]] && source /home/rosuser/catkin_ws/devel/setup.bash" /ros_entrypoint.sh > /ros_entrypoint.sh

# add user
RUN useradd -ms /bin/bash rosuser
USER rosuser

# init workspace
WORKDIR /home/rosuser
RUN mkdir catkin_ws/src -p
WORKDIR catkin_ws/src
RUN bash /ros_entrypoint.sh catkin_init_workspace

# download & compile sources
RUN git clone https://github.com/yi1306c12/khr_ros khr_ros
#RUN bash /ros_entrypoint.sh catkin_create_pkg khr_ros std_msgs std_srvs rospy roscpp
WORKDIR /home/rosuser/catkin_ws
RUN bash /ros_entrypoint.sh catkin_make


# set home directory
USER rosuser
WORKDIR /home/rosuser
RUN echo "$ i [[ -f  /home/rosuser/catkin_ws/devel/setup.bash ]] && source /home/rosuser/catkin_ws/devel/setup.bash" > .bashrc
