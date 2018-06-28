#include "ros/ros.h"
#include "khr_ros/AddTwoInts.h"
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
  ros::ServiceClient client = n.serviceClient<khr_ros::AddTwoInts>("add_two_ints");
  khr_ros::AddTwoInts srv;
  for(int i = 1; i < 4; ++i)
  {
    srv.request.action[i] = atoi(argv[i]);
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