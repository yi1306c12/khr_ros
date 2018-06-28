#include "ros/ros.h"
#include "khr_ros/khr_srv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 4)
  {
    ROS_INFO("usage: add_two_ints_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<khr_ros::khr_srv>("khr_srv");
  khr_ros::khr_srv srv;
  for(int i = 0; i < 3; ++i)
  {
    srv.request.action[i] = atoi(argv[i+1]);
  }

  if (client.call(srv))
  {
    ROS_INFO("Sum: %d,%d,%d", srv.response.angle[0],srv.response.angle[1],srv.response.angle[2]);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}