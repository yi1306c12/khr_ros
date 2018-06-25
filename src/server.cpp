#include "ros/ros.h"
#include "khr_ros/AddTwoInts.h"

//http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
struct AddTwoInts_service
{
public:
  bool add(khr_ros::AddTwoInts::Request  &req,
          khr_ros::AddTwoInts::Response &res)
  {
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  AddTwoInts_service ati = AddTwoInts_service();

  ros::ServiceServer service = n.advertiseService("add_two_ints", &AddTwoInts_service::add, &ati);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}