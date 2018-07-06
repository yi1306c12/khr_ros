#include "ros/ros.h"
#include "khr_ros/single_servo.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_servo_client");
  if (argc != 3)
  {
    ROS_INFO("usage: single_servo_client ID ROTATE");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<khr_ros::single_servo>("single_servo");
  khr_ros::single_servo srv;
  srv.request.id = atoi(argv[1]);
  srv.request.rotation = atoi(argv[2]);

  if (client.call(srv))
  {
    ROS_INFO("servo: id:%d\trotate:%d",srv.request.id,srv.request.rotation);
  }
  else
  {
    ROS_ERROR("Failed to call service single_servo");
    return 1;
  }

  return 0;
}