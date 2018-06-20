pkgname=$1

source /opt/ros/`rosversion -d`/setup.bash
source /catkin_ws/devel/setup.bash

# create & make pkg
cd /catkin_ws/src
catkin_create_pkg $pkgname std_msgs std_srvs rospy roscpp
cp /root/$pkgname . -r
cd /catkin_ws
catkin_make