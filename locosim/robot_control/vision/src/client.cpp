#include "ros/ros.h"
#include <vision/vision.h>
#include <cstdlib>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vision_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<vision::vision>("vision_service");
  vision::vision srv;
  
  if (client.call(srv)){
    ROS_INFO("#n res: %d", (int)srv.response.n_res);
  }else{
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
