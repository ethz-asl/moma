#include <moma_utils_cpp/giraffe_moveit.h>
#include <ros/ros.h>
// #include "ros/service_client.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "go_to_safe_pos");
  ros::NodeHandle nh;

  GiraffeMoveItUtils giraffe_moveit(nh);

  // if (giraffe_moveit.goToSafePos()) {
  //   ROS_INFO("Panda successfully in safe position");
  // } else {
  //   ROS_WARN("Panda did not make it to safe position");
  // }

  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("go_to_safe_pos");
  std_srvs::Trigger srv;

  client.call(srv);

  ros::waitForShutdown(); // keep node running

  return 0;
}