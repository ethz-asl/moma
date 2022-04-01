#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <panda_control/cartesian_velocity_controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace panda_control {

bool CartesianVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                       ros::NodeHandle &node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("CartesianVelocityController: Exception getting state handle: " << e.what());
    return false;
  }

  // Load parameters
  if (!node_handle.getParam("max_duration_between_commands", max_duration_between_commands_)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter max_duration_between_commands");
    return false;
  }
  if (!node_handle.getParam("rate_limiting/linear/velocity", max_velocity_linear_)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter rate_limiting/linear/velocity");
    return false;
  }
  if (!node_handle.getParam("rate_limiting/linear/acceleration", max_acceleration_linear_)) {
    ROS_ERROR(
        "CartesianVelocityController: Could not get parameter rate_limiting/acc/acceleration");
    return false;
  }
  if (!node_handle.getParam("rate_limiting/linear/jerk", max_jerk_linear_)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter rate_limiting/linear/jerk");
    return false;
  }
  if (!node_handle.getParam("rate_limiting/angular/velocity", max_velocity_angular_)) {
    ROS_ERROR(
        "CartesianVelocityController: Could not get parameter rate_limiting/angular/velocity");
    return false;
  }
  if (!node_handle.getParam("rate_limiting/angular/acceleration", max_acceleration_angular_)) {
    ROS_ERROR(
        "CartesianVelocityController: Could not get parameter rate_limiting/acc/acceleration");
    return false;
  }
  if (!node_handle.getParam("rate_limiting/angular/jerk", max_jerk_angular_)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter rate_limiting/angular/jerk");
    return false;
  }

  velocity_command_subscriber_ = node_handle.subscribe(
      "set_command", 10, &CartesianVelocityController::cartesian_velocity_cb, this);

  return true;
}

void CartesianVelocityController::starting(const ros::Time & /* time */) {
  time_since_last_command_ = ros::Duration(0.0);
  desired_velocity_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

void CartesianVelocityController::cartesian_velocity_cb(const geometry_msgs::Twist::ConstPtr &msg) {
  // Callback for ROS message
  desired_velocity_command_[0] = msg->linear.x;
  desired_velocity_command_[1] = msg->linear.y;
  desired_velocity_command_[2] = msg->linear.z;
  desired_velocity_command_[3] = msg->angular.x;
  desired_velocity_command_[4] = msg->angular.y;
  desired_velocity_command_[5] = msg->angular.z;

  time_since_last_command_ = ros::Duration(0.0);
}

void CartesianVelocityController::update(const ros::Time & /* time */,
                                         const ros::Duration &period) {
  time_since_last_command_ += period;
  if (time_since_last_command_.toSec() > max_duration_between_commands_) {
    desired_velocity_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  auto state = state_handle_->getRobotState();
  auto velocity_command =
      franka::limitRate(max_velocity_linear_, max_acceleration_linear_, max_jerk_linear_,
                        max_velocity_angular_, max_acceleration_angular_, max_jerk_angular_,
                        desired_velocity_command_, state.O_dP_EE_c, state.O_ddP_EE_c);

  velocity_cartesian_handle_->setCommand(velocity_command);
}

void CartesianVelocityController::stopping(const ros::Time & /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace panda_control

PLUGINLIB_EXPORT_CLASS(panda_control::CartesianVelocityController,
                       controller_interface::ControllerBase)
