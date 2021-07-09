#include "moma_joint_space_controller/controller.h"
#include "moma_msgs/JointResult.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace moma_controllers {

bool JointSpaceController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh) {

  if (!controller_nh.param<std::vector<std::string>>("joint_names", joint_names_, {})){
    ROS_ERROR("Failed to get joint_names param");
    return false;
  }
  n_joints_ = joint_names_.size();

  if (!controller_nh.getParam("simulation", sim_)) {
    ROS_ERROR_STREAM("Failed to get simulation param");
    return false;
  }

  if (!controller_nh.getParam("lower_limit", lower_limit_) || lower_limit_.size() != n_joints_) {
    ROS_ERROR_STREAM("Failed to get lower_limit or invalid param");
    return false;
  }

  if (!controller_nh.getParam("upper_limit", upper_limit_) || upper_limit_.size() != n_joints_) {
    ROS_ERROR_STREAM("Failed to get upper_limit or invalid param");
    return false;
  }

  if (!controller_nh.getParam("max_velocity", max_velocity_) || max_velocity_ < 0) {
    ROS_ERROR_STREAM("Failed to get max_velocity or invalid param");
    return false;
  }

  double max_acceleration;
  if (!controller_nh.getParam("max_acceleration", max_acceleration) || max_acceleration < 0) {
    ROS_ERROR_STREAM("Failed to get max_acceleration or invalid param");
    return false;
  }

  if (!controller_nh.getParam("gain", gain_) || gain_ < 0) {
    ROS_ERROR_STREAM("Failed to get gain or invalid param");
    return false;
  }

  if (!controller_nh.getParam("tolerance", tolerance_) || tolerance_ < 0) {
    ROS_ERROR_STREAM("Failed to get tolerance or invalid param");
    return false;
  }

  if (!controller_nh.getParam("/robot_description", robot_description_) ||
      robot_description_.empty()) {
    ROS_ERROR_STREAM("Could not find param /robot_description or invalid param");
    return false;
  }

  q_.setZero(n_joints_);
  qd_.setZero(n_joints_);
  trajectory_available_ = false;

  generator_ = std::make_unique<TrajectoryGenerator>(max_velocity_, max_acceleration,
                                                     n_joints_);  // max vel, max acc, size
  joint_desired_ = Eigen::VectorXd::Zero(n_joints_);
  joint_current_ = Eigen::VectorXd::Zero(n_joints_);

  trajectory_subscriber_ =
      controller_nh.subscribe("/joint_space_controller/goal", 1,
                              &JointSpaceController::joint_callback, this);

  action_server_ = std::make_unique<ActionServer>(
      controller_nh, "/joint_space_controller_server",
      boost::bind(&JointSpaceController::execute_callback, this, _1), false);

  // Init specialized command handles
  if (!add_command_handles(hw)) return false;

  // Custom initialization
  if (sim_) {
    model_ = std::make_unique<rc::RobotWrapper>();
    model_->initFromXml(robot_description_);

    for (size_t i = 0; i < n_joints_; i++) {
      control_toolbox::Pid pid;
      if (!pid.init(
              ros::NodeHandle("/joint_space_controller/pid_gains/" + joint_names_[i]), false)) {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
        return false;
      }
      pid_controllers_.push_back(pid);
    }

    // implicitly set to zero the gripper joints if present and other joints in the chain
    q_.setZero(model_->getDof());
    qd_.setZero(model_->getDof());
  }

  read_state();
  position_command_ = Eigen::VectorXd::Zero(n_joints_);
  velocity_command_ = Eigen::VectorXd::Zero(n_joints_);
  return true;
}

bool JointSpaceController::add_command_handles(hardware_interface::RobotHW* hw) {
  if (sim_){
    auto command_interface = hw->get<hardware_interface::EffortJointInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get command interface");
      return false;
    }

    for (size_t i = 0; i < n_joints_; i++) {
      joint_handles_.push_back(command_interface->getHandle(joint_names_[i]));
    }
  }
  else{
    auto command_interface = hw->get<hardware_interface::VelocityJointInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get command interface");
      return false;
    }

    for (size_t i = 0; i < n_joints_; i++) {
      joint_handles_.push_back(command_interface->getHandle(joint_names_[i]));
    }
  }

  return true;
}

void JointSpaceController::update(const ros::Time& time, const ros::Duration& period) {
  if (trajectory_available_) {
    Eigen::VectorXd joint_desired_now;
    {
      std::unique_lock<std::mutex> lock(generator_mutex_);
      joint_desired_now = generator_->get_next_point(ros::Time::now().toSec());
    }

    read_state();
    bool all_reached = true;
    for (int i = 0; i < n_joints_; i++) {
      double joint_error, velocity_command;
      angles::shortest_angular_distance_with_large_limits(
          q_[i], joint_desired_now(i), lower_limit_[i], upper_limit_[i], joint_error);
      std::stringstream ss;
      ss << "Joint current=" << q_[i] << std::endl;
      ss << "joint desired=" << joint_desired_now(i) << std::endl;
      ss << "lower limit=" << lower_limit_[i] << std::endl;
      ss << "upper limit=" << upper_limit_[i] << std::endl;
      ss << "error=" << joint_error << std::endl;
      ROS_DEBUG_STREAM_THROTTLE(1.0, ss.str());

      if (abs(joint_error) < tolerance_) {
        position_command_[i] = q_[i];
        velocity_command_[i] = 0.0;
      } else {
        position_command_[i] = q_[i] + joint_error;
        velocity_command_[i] =
            std::min(std::max(gain_ * joint_error, -max_velocity_), max_velocity_);
        all_reached = false;
      }
      success_ = all_reached;
    }
  } else {
    velocity_command_.setZero();
  }
  write_command();
}

void JointSpaceController::write_command() {
  if (!sim_) {
    for (int i = 0; i < n_joints_; i++) {
      joint_handles_[i].setCommand(velocity_command_[i]);
    }
  } 
  else {
    read_state();
    model_->updateState(q_, qd_);
    model_->computeAllTerms();
    Eigen::VectorXd gravity_and_coriolis = model_->getNonLinearTerms().head(n_joints_);

    static Eigen::VectorXd pd_term(n_joints_);
    static Eigen::VectorXd position_error(n_joints_);
    static Eigen::VectorXd velocity_error(n_joints_);

    for (int i = 0; i < n_joints_; i++) {
      angles::shortest_angular_distance_with_large_limits(
          q_(i), position_command_(i), lower_limit_[i], upper_limit_[i], position_error(i));
      velocity_error(i) = 0.0 - qd_(i);
      pd_term(i) = pid_controllers_[i].computeCommand(position_error(i), velocity_error(i),
                                                      ros::Duration(0.001));
    }

    Eigen::VectorXd tau = model_->getInertia().block(0, 0, n_joints_, n_joints_) * pd_term + gravity_and_coriolis;
    ROS_DEBUG_STREAM_THROTTLE(
        1.0,
        "position current: " << q_.transpose() << std::endl
                             << "position desired: " << position_command_.transpose() << std::endl
                             << "position error  : " << position_error.transpose() << std::endl
                             << "velocity error  : " << velocity_error.transpose() << std::endl);
    for (int i = 0; i < n_joints_; i++) {
      joint_handles_[i].setCommand(tau(i));
    }
  }
}

void JointSpaceController::stopping(const ros::Time& time) {
  cleanup();
  action_server_->shutdown();
}

void JointSpaceController::compute_profile(const Eigen::VectorXd& goal) {
  // the next desired point is the closest within the limits
  double delta = 0;
  read_state();
  for (int i = 0; i < n_joints_; i++) {
    angles::shortest_angular_distance_with_large_limits(q_[i], goal[i], lower_limit_[i],
                                                        upper_limit_[i], delta);

    joint_desired_[i] = q_[i] + delta;
    joint_current_[i] = q_[i];
  }
  ROS_INFO_STREAM("Received new joint target configuration."
                  << std::endl
                  << "Current is: " << joint_current_.transpose() << std::endl
                  << "Desired is : " << goal.transpose() << std::endl
                  << "Correct desired is: " << joint_desired_.transpose());
  {
    std::unique_lock<std::mutex> lock(generator_mutex_);
    generator_->compute(joint_current_, joint_desired_, ros::Time::now().toSec());
  }

  ROS_INFO("Computed new velocity profile.");
  trajectory_available_ = true;
}

void JointSpaceController::read_state() {
  for (int i = 0; i < n_joints_; i++) {
    q_(i) = joint_handles_[i].getPosition();
    qd_(i) = joint_handles_[i].getVelocity();
  }
}

void JointSpaceController::cleanup() {
  if (sim_) return;

  for (size_t i = 0; i < n_joints_; i++) {
    joint_handles_[i].setCommand(0.0);
  }
};

void JointSpaceController::execute_callback(
    const moma_msgs::JointGoalConstPtr& goal) {
  if (goal->position.size() != n_joints_) {
    ROS_ERROR_STREAM(
        "Target joint configuration message has the wrong size: " << goal->position.size());
    return;
  }

  Eigen::VectorXd joint_desired(n_joints_);
  for (int i = 0; i < n_joints_; i++) joint_desired[i] = goal->position[i];
  compute_profile(joint_desired);

  ros::Rate r(1);
  moma_msgs::JointResult result;
  result.success = false;
  success_ = false;

  // start executing the action
  while (ros::ok()) {
    // check that preempt has not been requested by the client
    if (action_server_->isPreemptRequested()) {
      ROS_INFO("JointTrajectoryController preempted");
      action_server_->setPreempted();
      trajectory_available_ = false;
      return;
    }

    if (success_) {
      result.success = true;
      action_server_->setSucceeded(result);
      ROS_INFO("[JointTrajectoryController::execute_callback]: Goal reached.");
      trajectory_available_ = false;

      // freeze robot
      read_state();
      position_command_ = q_;
      return;
    }
    r.sleep();
  }
}

void JointSpaceController::joint_callback(const sensor_msgs::JointStateConstPtr& msg) {
  if (msg->position.size() != n_joints_) {
    ROS_ERROR_STREAM(
        "Target joint configuration message has the wrong size: " << msg->position.size());
    return;
  }

  Eigen::VectorXd joint_desired(n_joints_);
  for (int i = 0; i < n_joints_; i++) joint_desired[i] = msg->position[i];
  compute_profile(joint_desired);
}
}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::JointSpaceController, controller_interface::ControllerBase)
