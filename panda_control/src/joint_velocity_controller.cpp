// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Modified by members of ASL, ETH Zurich
#include <panda_control/joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_control {

bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityController: Could not parse joint names.");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityController: Could not get state interface from hardware.");
    return false;
  }

  velocity_command_subscriber_ = node_handle.subscribe("command",
                                                       10,
                                                       &JointVelocityController::joint_velocity_cb,
                                                       this);
  return true;
}

void JointVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  desired_velocity_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

void JointVelocityController::joint_velocity_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (msg->data.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityController: Received joint command with less than 7 floats!");
    return;
  }
  for (int i = 0; i < 7; ++i) {desired_velocity_command_[i] = msg->data[i];}
}

void JointVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  for (int i = 0; i < velocity_joint_handles_.size(); ++i) {
    velocity_joint_handles_[i].setCommand(desired_velocity_command_[i]);
  }
  
}

void JointVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace panda_control

PLUGINLIB_EXPORT_CLASS(panda_control::JointVelocityController,
                       controller_interface::ControllerBase)
