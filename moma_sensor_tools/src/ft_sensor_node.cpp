//
// Created by giuseppe on 25.01.21.
//

#include "moma_sensor_tools/ft_sensor.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "force_torque_sensor_node");
  ros::NodeHandle nh("~");

  moma_sensor_tools::ForceTorqueSensor ft_sensor(nh);
  if (!ft_sensor.init()) {
    ROS_ERROR("Failed to initialize the force torque sensor node ... exiting.");
    return 0;
  }

  ros::Rate rate(200);
  while (ros::ok()) {
    ft_sensor.update();
    rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO("force_torque_sensor_node killed.");
  return 0;
}
