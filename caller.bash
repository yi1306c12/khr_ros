source /opt/ros/`rosversion -d`/setup.bash
source /catkin_ws/devel/setup.bash

bash catkin_build.bash "server_caller"

# run caller programs
rosrun roscpp_tutorials talker