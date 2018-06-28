#include "ros/ros.h"
#include "khr_ros/AddTwoInts.h"

#include<vector>
using std::vector;

//http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
struct AddTwoInts_service
{
  int N;
public:
  AddTwoInts_service(const int n)
  {
    N = n;
  }
  bool add(khr_ros::AddTwoInts::Request  &req,
          khr_ros::AddTwoInts::Response &res)
  {
    for(int i = 0;i < N;++i)
    {
      res.angle[i] = req.action[i]*req.action[i];
    }
    ROS_INFO("request: %d,%d,%d", req.action[0], req.action[1], req.action[2]);
    ROS_INFO("sending back response: [%d,%d,%d]", res.angle[0],res.angle[1],res.angle[2]);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  AddTwoInts_service ati = AddTwoInts_service(3);

  ros::ServiceServer service = n.advertiseService("add_two_ints", &AddTwoInts_service::add, &ati);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}