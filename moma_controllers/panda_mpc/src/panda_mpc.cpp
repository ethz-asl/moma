//
// Created by giuseppe on 22.01.21.
//

#include <panda_mpc/panda_mpc.hpp>

#include <memory>
#include <chrono>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <angles/angles.h>

#include <franka/robot_state.h>
#include <tf2/transform_datatypes.h>
//#include <geometry_msgs/TransformStamped.h>

namespace moma_controllers {

constexpr double PandaMpcController::kDeltaTauMax;

bool PandaMpcController::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle,
                                   ros::NodeHandle& controller_nh) {
  if (!init_parameters(controller_nh)) return false;
  if (!init_common_interfaces(robot_hw)) return false;
  if (!init_franka_interfaces(robot_hw)) return false;

  std::string arm_description;
  if (!node_handle.getParam("/arm_description", arm_description)) {
    ROS_ERROR("Failed to retrieve /arm_description from param server.");
    return false;
  }
  robot_model_ = std::make_unique<rc::RobotWrapper>();
  robot_model_->initFromXml(arm_description);

  // the pinocchio model contains only the arm and not the full base which
  // would require to maintain the full chain of links
  position_current_model_.setZero(robot_model_->getDof());
  velocity_current_model_.setZero(robot_model_->getDof());
  
  position_current_ = Eigen::VectorXd::Zero(ocs2::mobile_manipulator::STATE_DIM(armInputDim_));
  velocity_current_ = Eigen::VectorXd::Zero(ocs2::mobile_manipulator::STATE_DIM(armInputDim_));
  ROS_INFO("[PandaMpc::init] robot model successfully initialized");

  mpc_controller_ = std::unique_ptr<moma_controllers::MpcController<armInputDim_>>(new moma_controllers::MpcController<armInputDim_>(controller_nh));
  if (!mpc_controller_->init()) {
    ROS_ERROR("Failed to initialize the MPC controller");
    return false;
  }

  ROS_INFO("Controller successfully initialized!");
  started_ = false;
  return true;
}

bool PandaMpcController::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("simulation", sim_)) {
    ROS_ERROR("PandaMpcController: Could not read parameter simulation");
    return false;
  }

  /*if (!node_handle.getParam("world_frame", world_frame_)) {
    ROS_ERROR("PandaMpcController: Could not read parameter world_frame");
    return false;
  }

  if (!node_handle.getParam("base_link", base_link_)) {
    ROS_ERROR("PandaMpcController: Could not read parameter base_link");
    return false;
  }*/

  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("PandaMpcController: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != armInputDim_) {
    ROS_ERROR(
        "PandaMpcController: Invalid or no joint_names parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("odom_topic", odom_topic_)) {
    ROS_WARN("PandaMpcController: Could not read parameter odom_topic, ignoring");
  } else {
    odom_sub_ = node_handle.subscribe(odom_topic_, 1,
                                      &PandaMpcController::odom_callback, this);
  }

  if (!node_handle.getParam("command_base_topic", command_base_topic_)) {
    ROS_WARN("PandaMpcController: Could not read parameter command_base_topic, ignoring");
  } else {
    command_base_pub_ = node_handle.advertise<geometry_msgs::Twist>(command_base_topic_, 1);
  }

  for (size_t i = 0; i < armInputDim_; i++) {
    if (!arm_pid_controllers_[i].init(ros::NodeHandle(node_handle, node_handle.getNamespace() + "/gains/" + joint_names_[i]),
                                      false)) {
      ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
      return false;
    }
  }

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM(
        "PandaMpcController: coriolis_factor not found. Defaulting to "
        << coriolis_factor_);
  }

  if (!node_handle.getParam("measurement_trust_factor", measurement_trust_factor_)) {
    ROS_INFO_STREAM(
        "PandaMpcController: measurement_trust_factor not found. Defaulting to "
            << measurement_trust_factor_);
  }

  ROS_INFO(
      "[PandaMpcController::init_parameters]: Parameters successfully "
      "initialized.");
  return true;
}

bool PandaMpcController::init_common_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  // Effort interface for compliant control
  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PandaMpcController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < armInputDim_; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaMpcController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
  ROS_INFO(
      "[PandaMpcController::init_common_interfaces]: Interfeces "
      "successfully initialized.");
  return true;
}

bool PandaMpcController::init_franka_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  if (sim_) return true;

  // Model interface: returns kino-dynamic properties of the manipulator
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PandaMpcController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaMpcController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  // Get state interface.
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PandaMpcController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaMpcController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }
  ROS_INFO(
      "[PandaMpcController::init_franka_interfaces]: Interfaces "
      "successfully initialized.");
  return true;
}


void PandaMpcController::write_command(){

  // Send motion commands to base
  if (command_base_pub_) {
    command_base_pub_.publish(base_velocity_command_);
  }

  for (size_t i = 0; i < armInputDim_; i++) {
    joint_handles_[i].setCommand(arm_tau_(i));
  }
}

void PandaMpcController::starting(const ros::Time& time) {
  if (started_){
    ROS_INFO("[PandaMpcController::starting] Controller already started.");
    return;
  };
  read_state();

  ROS_DEBUG_STREAM("[PandaMpcController::starting] Starting with current joint position: " << position_current_.transpose());
  mpc_controller_->start(position_current_);
  position_integral_ = position_current_;

  started_ = true;
  last_start_time_ = ros::Time::now();
  ROS_INFO("[PandaMpcController::starting] Controller started!");
}


void PandaMpcController::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
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


void PandaMpcController::read_state(){
  /*// Alternative reading of odometry
  geometry_msgs::TransformStamped world_to_base;
  try {
    world_to_base =
        tfBuffer_.lookupTransform(base_link_, world_frame_, ros::Time(0));
    position_current_(0) = world_to_base.transform.translation.x;
    position_current_(1) = world_to_base.transform.translation.y;
    tf2::Quaternion rot(world_to_base.transform.rotation.x, world_to_base.transform.rotation.y,
                        world_to_base.transform.rotation.z, world_to_base.transform.rotation.w);
    position_current_(2) = rot.getAngle();
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Could not read world to base transform");
  }*/


  for (size_t i = 0; i < armInputDim_; i++) {
    position_current_model_(i) = joint_handles_[i].getPosition();
    position_current_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM) = joint_handles_[i].getPosition();
    velocity_current_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM) = velocity_current_(i) * (1-measurement_trust_factor_) + measurement_trust_factor_ * joint_handles_[i].getVelocity();
  }
}

void PandaMpcController::compute_command(const ros::Duration& period) {
  position_command_ = mpc_controller_->getPositionCommand();
  velocity_command_ = mpc_controller_->getVelocityCommand();
  position_integral_ += velocity_command_ * period.toSec();

  // x, y error
  assert(ocs2::mobile_manipulator::BASE_INPUT_DIM == 3);
  for (int i = 0; i < ocs2::mobile_manipulator::BASE_INPUT_DIM; i++){
    position_error_(i) = position_integral_(i) - position_current_(i);
  }
  // Do not feedforward velocity to add damping
  for (int i = ocs2::mobile_manipulator::BASE_INPUT_DIM; i < ocs2::mobile_manipulator::STATE_DIM(armInputDim_); i++){
    position_error_(i) = angles::shortest_angular_distance(position_current_(i), position_integral_(i));
  }
  for (int i = 0; i < ocs2::mobile_manipulator::INPUT_DIM(armInputDim_); i++){
    velocity_error_(i) = /*velocity_command_(i)*/ 0.0 - velocity_current_(i);
  }

  if (sim_) { 
    robot_model_->updateState(position_current_model_, velocity_current_model_);
    robot_model_->computeAllTerms();
    arm_gravity_and_coriolis_ = robot_model_->getNonLinearTerms();
  }
  else {

    // Panda already compensates for gravity internally
    robot_state_ = state_handle_->getRobotState();
    coriolis_ = model_handle_->getCoriolis();
    for(int i=0; i<armInputDim_; i++){
      arm_gravity_and_coriolis_(i) = coriolis_factor_ * coriolis_[i];
    }
  }

  assert(ocs2::mobile_manipulator::BASE_INPUT_DIM == 3);
  base_velocity_command_.linear.x =  velocity_command_(0);
  base_velocity_command_.linear.y =  velocity_command_(1);
  base_velocity_command_.angular.z = velocity_command_(2);

  for (int i = 0; i < armInputDim_; i++){
    arm_tau_(i) = arm_pid_controllers_[i].computeCommand(position_error_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM),
                                                         velocity_error_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM), period)
                                                     + arm_gravity_and_coriolis_(i);
  }
  // Maximum torque difference with a sampling rate of 1 kHz. The maximum
  // torque rate is 1000 * (1 / sampling_time).
  if (!sim_){
    saturate_torque_rate(robot_state_.tau_J_d);
  }
}

void PandaMpcController::update(const ros::Time& time,
                                     const ros::Duration& period) {
  read_state();
  mpc_controller_->update(time, position_current_);
  compute_command(period);
  write_command();
}

void PandaMpcController::stopping(const ros::Time& time) {
  if ((ros::Time::now() - last_start_time_).toSec() < 1) {
    ROS_WARN("[PandaMpcController::stopping] Ignoring controller stop");
    return;
  }
  if (!started_){
    ROS_WARN("[PandaMpcController::stopping] Controller already stopped.");
    return;
  };
  ROS_INFO("[PandaMpcController::stopping] Stopping Mpc Controller!");
  mpc_controller_->stop();
  started_ = false;
}

void PandaMpcController::saturate_torque_rate(
    const std::array<double, armInputDim_>& tau_J_d)
{
  double delta;
  for (int i = 0; i < armInputDim_; i++) {
    delta = arm_tau_(i) - tau_J_d[i];
    arm_tau_(i) = tau_J_d[i] +
        std::max(std::min(delta, kDeltaTauMax), -kDeltaTauMax);
  }
}

}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::PandaMpcController,
                       controller_interface::ControllerBase)