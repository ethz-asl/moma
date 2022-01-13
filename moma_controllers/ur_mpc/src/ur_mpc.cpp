//
// Created by julian on 20.12.21.
//

#include <ur_mpc/ur_mpc.hpp>

#include <memory>
#include <chrono>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <angles/angles.h>

#include <tf2/transform_datatypes.h>

namespace moma_controllers {

bool UrMpcController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
  if (!init_parameters(nh)) return false;
  if (!init_common_interfaces(hw)) return false;

  arm_vel_buffer_.writeFromNonRT(std::vector<double>(armInputDim_, 0.0));

  position_current_ = Eigen::VectorXd::Zero(ocs2::mobile_manipulator::STATE_DIM(armInputDim_));
  velocity_current_ = Eigen::VectorXd::Zero(ocs2::mobile_manipulator::STATE_DIM(armInputDim_));
  ROS_INFO("[UrMpc::init] robot model successfully initialized");

  mpc_controller_ = std::unique_ptr<moma_controllers::MpcController<armInputDim_>>(new moma_controllers::MpcController<armInputDim_>(nh));
  if (!mpc_controller_->init()) {
    ROS_ERROR("Failed to initialize the MPC controller");
    return false;
  }

  ROS_INFO("Controller successfully initialized!");
  started_ = false;
  return true;
}

bool UrMpcController::init_parameters(ros::NodeHandle& nh) {
  if (!nh.getParam("joint_names", joint_names_) ||
      joint_names_.size() != armInputDim_) {
    ROS_ERROR(
        "UrMpcController: Invalid or no joint_names parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }

  if (!nh.getParam("odom_topic", odom_topic_)) {
    ROS_WARN("UrMpcController: Could not read parameter odom_topic, ignoring");
  } else {
    odom_sub_ = nh.subscribe(odom_topic_, 1,
                             &UrMpcController::odom_callback, this);
  }

  if (!nh.getParam("command_base_topic", command_base_topic_)) {
    ROS_WARN("UrMpcController: Could not read parameter command_base_topic, ignoring");
  } else {
    command_base_pub_ = nh.advertise<geometry_msgs::Twist>(command_base_topic_, 1);
  }

  for (size_t i = 0; i < armInputDim_; i++) {
    if (!arm_pid_controllers_[i].init(ros::NodeHandle(nh, nh.getNamespace() + "/gains/" + joint_names_[i]),
                                      false)) {
      ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
      return false;
    }
  }

  if (!nh.getParam("measurement_trust_factor", measurement_trust_factor_)) {
    ROS_INFO_STREAM(
        "UrMpcController: measurement_trust_factor not found. Defaulting to "
            << measurement_trust_factor_);
  }

  ROS_INFO(
      "[UrMpcController::init_parameters]: Parameters successfully "
      "initialized.");
  return true;
}

bool UrMpcController::init_common_interfaces(hardware_interface::VelocityJointInterface* hw) {
  for(unsigned int i = 0; i < armInputDim_; ++i)
  {
    try
    {
      joint_handles_.push_back(hw->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }
  ROS_INFO(
      "[UrMpcController::init_common_interfaces]: Interfeces "
      "successfully initialized.");
  return true;
}


void UrMpcController::write_command() {

  // Send motion commands to base
  if (command_base_pub_) {
    command_base_pub_.publish(base_velocity_command_);
  }

  std::vector<double>& arm_vel = *arm_vel_buffer_.readFromRT();
  for (size_t i = 0; i < armInputDim_; i++) {
    joint_handles_[i].setCommand(arm_vel[i]);
  }
}

void UrMpcController::starting(const ros::Time& time) {
  if (started_) {
    ROS_INFO("[UrMpcController::starting] Controller already started.");
    return;
  }
  // Start controller with 0.0 velocities
  arm_vel_buffer_.readFromRT()->assign(armInputDim_, 0.0);
  read_state();

  ROS_DEBUG_STREAM("[UrMpcController::starting] Starting with current joint position: " << position_current_.transpose());
  mpc_controller_->start(position_current_.head<ocs2::mobile_manipulator::STATE_DIM(armInputDim_)>());
  position_integral_ = position_current_;

  started_ = true;
  last_start_time_ = ros::Time::now();
  ROS_INFO("[UrMpcController::starting] Controller started!");
}


void UrMpcController::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  assert(ocs2::mobile_manipulator::BASE_INPUT_DIM == 3);
  position_current_(0) = msg->pose.pose.position.x;
  position_current_(1) = msg->pose.pose.position.y;
  tf2::Quaternion rot(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(rot);
  const tf2::Vector3 ix = mat.getColumn(0);
  const double theta  = std::atan2(ix.y(), ix.x());
  position_current_(2) = theta;
  velocity_current_(0) = msg->twist.twist.linear.x;
  velocity_current_(1) = msg->twist.twist.linear.y;
  velocity_current_(2) = msg->twist.twist.angular.z;
}


void UrMpcController::read_state() {
  for (size_t i = 0; i < armInputDim_; i++) {
    position_current_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM) = joint_handles_[i].getPosition();
    velocity_current_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM) = velocity_current_(i) * (1-measurement_trust_factor_) + measurement_trust_factor_ * joint_handles_[i].getVelocity();
  }
}

void UrMpcController::compute_command(const ros::Duration& period) {
  position_command_ = mpc_controller_->getPositionCommand();
  velocity_command_ = mpc_controller_->getVelocityCommand();
  position_integral_ += velocity_command_ * period.toSec();

  // x, y error
  assert(ocs2::mobile_manipulator::BASE_INPUT_DIM == 3);
  for (int i = 0; i < ocs2::mobile_manipulator::BASE_INPUT_DIM; i++) {
    position_error_(i) = position_integral_(i) - position_current_(i);
  }
  // Do not feedforward velocity to add damping
  for (int i = ocs2::mobile_manipulator::BASE_INPUT_DIM; i < ocs2::mobile_manipulator::STATE_DIM(armInputDim_); i++) {
    position_error_(i) = angles::shortest_angular_distance(position_current_(i), position_integral_(i));
  }
  for (int i = 0; i < ocs2::mobile_manipulator::INPUT_DIM(armInputDim_); i++) {
    velocity_error_(i) = /*velocity_command_(i)*/ 0.0 - velocity_current_(i);
  }

  assert(ocs2::mobile_manipulator::BASE_INPUT_DIM == 3);
  base_velocity_command_.linear.x =  velocity_command_(0);
  base_velocity_command_.linear.y =  velocity_command_(1);
  base_velocity_command_.angular.z = velocity_command_(2);

  std::vector<double> arm_vel(armInputDim_);
  for (int i = 0; i < armInputDim_; i++) {
    arm_vel[i] = arm_pid_controllers_[i].computeCommand(position_error_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM),
                                                        velocity_error_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM), period);
  }
  arm_vel_buffer_.writeFromNonRT(arm_vel);
}

void UrMpcController::update(const ros::Time& time, const ros::Duration& period) {
  read_state();
  //ROS_INFO_STREAM_THROTTLE(0.5, "State is: " << position_current_.head<ocs2::mobile_manipulator::STATE_DIM(armInputDim_)>());
  mpc_controller_->update(time, position_current_.head<ocs2::mobile_manipulator::STATE_DIM(armInputDim_)>());
  compute_command(period);
  write_command();
}

void UrMpcController::stopping(const ros::Time& time) {
  if ((ros::Time::now() - last_start_time_).toSec() < 1) {
    ROS_WARN("[UrMpcController::stopping] Ignoring controller stop");
    return;
  }
  if (!started_) {
    ROS_WARN("[UrMpcController::stopping] Controller already stopped.");
    return;
  }
  ROS_INFO("[UrMpcController::stopping] Stopping Mpc Controller!");
  mpc_controller_->stop();
  started_ = false;
}

}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::UrMpcController,
                       controller_interface::ControllerBase)