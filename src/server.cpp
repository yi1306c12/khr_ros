#include "ros/ros.h"
#include "khr_ros/khr_srv.h"

#include "kondo_servo.hpp"

#include <vector>
using std::vector;

const int N = 3;

//http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
struct khr_service
{
  kondo_servo servo[N];
public:

  khr_service(const vector<int>& id)
  {
    for(int i = 0; i < N; ++i)
    {
      &servo[i] = new kondo_servo(static_cast<unsigned char>(id));
    }
  }

  bool add(khr_ros::khr_srv::Request  &req,
          khr_ros::khr_srv::Response &res)
  {
    for(int i = 0;i < N;++i)
    {
//      res.angle[i] = req.action[i]*req.action[i];
      const unsigned short r = servo[i].rotate(static_cast<unsigned short>(req.action[i]));
      res.angle[i] = static_cast<int>(r);
    }
    ROS_INFO("request: %d,%d,%d", req.action[0], req.action[1], req.action[2]);
    ROS_INFO("sending back response: [%d,%d,%d]", res.angle[0],res.angle[1],res.angle[2]);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "khr_server");
  ros::NodeHandle n;

  khr_service ati = khr_service(3);

  ros::ServiceServer service = n.advertiseService("khr_srv", &khr_service::add, &ati);
  ROS_INFO("Ready khr.");
  ros::spin();

  return 0;
}