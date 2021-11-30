//
// Created by giuseppe on 22.01.21.
//

#include <panda_mpc/panda_mpc.hpp>

#include <cmath>
#include <memory>
#include <chrono>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <angles/angles.h>

#include <franka/robot_state.h>

namespace moma_controllers {

constexpr double PandaMpcController::kDeltaTauMax;

bool PandaMpcController::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle,
                                   ros::NodeHandle& controller_nh) {
  if (!init_parameters(controller_nh)) return false;
  if (!init_common_interfaces(robot_hw)) return false;
  if (!init_franka_interfaces(robot_hw)) return false;

  std::string arm_description;
  if (!node_handle.getParam("/ocs2_mpc/robot_description_ocs2", arm_description)) {
    ROS_ERROR("Failed to retrieve /ocs2_mpc/robot_description_ocs2 from param server.");
    return false;
  }
  robot_model_ = std::make_unique<rc::RobotWrapper>();
  robot_model_->initFromXml(arm_description);
  position_current_ = Eigen::VectorXd::Zero(robot_model_->getDof());
  velocity_current_ = Eigen::VectorXd::Zero(robot_model_->getDof());
  ROS_INFO("[PandaMpc::init] robot model successfully initialized");

  mpc_controller_ = std::unique_ptr<moma_controllers::MpcController>(new moma_controllers::MpcController(controller_nh));
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


  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("PandaMpcController: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != ocs2::mobile_manipulator::ARM_INPUT_DIM) {
    ROS_ERROR(
        "PandaMpcController: Invalid or no joint_names parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }

  for (size_t i = 0; i < ocs2::mobile_manipulator::ARM_INPUT_DIM; i++) {
    if (!pid_controllers_[i].init(ros::NodeHandle(node_handle, node_handle.getNamespace() + "/gains/" + joint_names_[i]),
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
  for (size_t i = 0; i < ocs2::mobile_manipulator::ARM_INPUT_DIM; ++i) {
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

  // TODO Send motion commands to base

  for (size_t i = 0; i < ocs2::mobile_manipulator::ARM_INPUT_DIM; i++) {
    joint_handles_[i].setCommand(arm_tau_(i));
  }
}

void PandaMpcController::starting(const ros::Time& time) {
  if (started_){
    ROS_INFO("[PandaMpcController::starting] Controller already started.");
  };
  read_state();

  ROS_DEBUG_STREAM("[PandaMpcController::starting] Starting with current joint position: " << position_current_.transpose());
  mpc_controller_->start(position_current_.head<ocs2::mobile_manipulator::STATE_DIM>());
  position_integral_ = position_current_;

  started_ = true;
  ROS_INFO("[PandaMpcController::starting] Controller started!");
}


void PandaMpcController::read_state(){
  static double alpha = 0.99;

  // TODO Add reading of odometry

  for (size_t i = 0; i < ocs2::mobile_manipulator::ARM_INPUT_DIM; i++) {
    position_current_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM) = joint_handles_[i].getPosition();
    velocity_current_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM) = velocity_current_(i) * (1-alpha) + alpha * joint_handles_[i].getVelocity();
  }
}

void PandaMpcController::compute_torque(const ros::Duration& period) {
  position_command_ = mpc_controller_->getPositionCommand();
  velocity_command_ = mpc_controller_->getVelocityCommand();
  position_integral_ += velocity_command_ * period.toSec();

  // Do not feedforward velocity to add damping
  for (int i = 0; i < ocs2::mobile_manipulator::STATE_DIM; i++){
    position_error_(i) = angles::shortest_angular_distance(position_current_(i), position_integral_(i));
  }
  for (int i = 0; i < ocs2::mobile_manipulator::INPUT_DIM; i++){
    velocity_error_(i) = /*velocity_command_(i)*/ 0.0 - velocity_current_(i);
  }

  if (sim_) {
    robot_model_->updateState(position_current_, Eigen::VectorXd::Zero(robot_model_->getDof()));
    robot_model_->computeAllTerms();
    arm_gravity_and_coriolis_ = robot_model_->getNonLinearTerms().head<ocs2::mobile_manipulator::ARM_INPUT_DIM>();
  }
  else {

    // Panda already compensates for gravity internally
    robot_state_ = state_handle_->getRobotState();
    coriolis_ = model_handle_->getCoriolis();
    for(int i=0; i<ocs2::mobile_manipulator::ARM_INPUT_DIM; i++){
      arm_gravity_and_coriolis_[i] = coriolis_factor_ * coriolis_[i];
    }
  }


  for (int i = 0; i < ocs2::mobile_manipulator::ARM_INPUT_DIM; i++){
    arm_tau_(i) = pid_controllers_[i].computeCommand(position_error_(i + ocs2::mobile_manipulator::BASE_INPUT_DIM),
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
  mpc_controller_->update(time, position_current_.head<ocs2::mobile_manipulator::STATE_DIM>());
  compute_torque(period);
  write_command();
}

void PandaMpcController::stopping(const ros::Time& time) {
  ROS_INFO("Stopping Mpc Controller!");
  mpc_controller_->stop();
}

void PandaMpcController::saturate_torque_rate(
    const std::array<double, ocs2::mobile_manipulator::ARM_INPUT_DIM>& tau_J_d)
{
  double delta;
  for (int i = 0; i < ocs2::mobile_manipulator::ARM_INPUT_DIM; i++) {
    delta = arm_tau_[i] - tau_J_d[i];
    arm_tau_[i] = tau_J_d[i] +
        std::max(std::min(delta, kDeltaTauMax), -kDeltaTauMax);
  }
}

}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::PandaMpcController,
                       controller_interface::ControllerBase)