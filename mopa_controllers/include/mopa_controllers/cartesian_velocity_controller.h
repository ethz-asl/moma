#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/rate_limiting.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace mopa_controllers {

class CartesianVelocityController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  void cartesian_velocity_cb(const geometry_msgs::Twist::ConstPtr& msg);

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  ros::Duration elapsed_time_;

  ros::Subscriber velocity_command_subscriber;

  std::array<double, 6> desired_velocity_command_;
  ros::Duration time_since_last_command;

  double max_duration_between_commands;
  double max_velocity_linear;
  double max_acceleration_linear;
  double max_jerk_linear;
  double max_velocity_angular;
  double max_acceleration_angular;
  double max_jerk_angular;
};

}  // namespace mopa_controllers
