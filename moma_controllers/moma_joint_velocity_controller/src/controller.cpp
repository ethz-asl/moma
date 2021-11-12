#include "moma_joint_velocity_controller/controller.h"
#include "moma_msgs/JointResult.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace moma_controllers {

bool JointVelocityController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
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

  if (!controller_nh.getParam("safety_margin", safety_margin_) || safety_margin_ < 0) {
    ROS_ERROR_STREAM("Failed to get safety_margin_ or invalid param");
    return false;
  }

  if (!controller_nh.getParam("max_velocity", max_velocity_) || max_velocity_ < 0) {
    ROS_ERROR_STREAM("Failed to get max_velocity or invalid param");
    return false;
  }

  if (!controller_nh.getParam("max_acceleration", max_acceleration_) || max_acceleration_ < 0) {
    ROS_ERROR_STREAM("Failed to get max_acceleration or invalid param");
    return false;
  }

  if (!controller_nh.getParam("max_deceleration", max_deceleration_) || max_deceleration_ < 0) {
    ROS_ERROR_STREAM("Failed to get max_deceleration or invalid param");
    return false;
  }

  if (!controller_nh.getParam("gain", gain_) || gain_ < 0) {
    ROS_ERROR_STREAM("Failed to get gain or invalid param");
    return false;
  }

  if (!controller_nh.getParam("/robot_description", robot_description_) ||
      robot_description_.empty()) {
    ROS_ERROR_STREAM("Could not find param /robot_description or invalid param");
    return false;
  }

  q_.setZero(n_joints_);
  qd_.setZero(n_joints_);

  velocity_available_ = false;
  velocity_desired_ = Eigen::VectorXd::Zero(n_joints_);
  velocity_subscriber_ =
      controller_nh.subscribe("/joint_velocity_controller/goal", 1,
                              &JointVelocityController::joint_callback, this);

  // Init specialized command handles
  if (!add_command_handles(hw)) return false;

  // Custom initialization
  if (sim_) {
    model_ = std::make_unique<rc::RobotWrapper>();
    model_->initFromXml(robot_description_);

    for (size_t i = 0; i < n_joints_; i++) {
      control_toolbox::Pid pid;
      if (!pid.init(
              ros::NodeHandle("/joint_velocity_controller/pid_gains/" + joint_names_[i]), false)) {
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
  position_command_ = q_.head(n_joints_);
  velocity_command_ = Eigen::VectorXd::Zero(n_joints_);
  ROS_INFO("Controller successfully initialized.");
  return true;
}

void JointVelocityController::starting(const ros::Time& time){
  read_state();
  position_command_ = q_.head(n_joints_);
  velocity_command_ = Eigen::VectorXd::Zero(n_joints_);
  ROS_INFO("Controller successfully started.");
}

bool JointVelocityController::add_command_handles(hardware_interface::RobotHW* hw) {
  if (sim_) {
    auto command_interface = hw->get<hardware_interface::EffortJointInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get command interface");
      return false;
    }

    for (size_t i = 0; i < n_joints_; i++) {
      joint_handles_.push_back(command_interface->getHandle(joint_names_[i]));
    }
  }
  else {
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

void JointVelocityController::update(const ros::Time& time, const ros::Duration& period) {
  if (velocity_available_) {
    const double dt = period.toSec();
    read_state();
    for (int i = 0; i < n_joints_; i++) {

      const double acc_dec = (velocity_desired_[i] < velocity_command_[i] && velocity_command_[i] > 0)
          || (velocity_desired_[i] > velocity_command_[i] && velocity_command_[i] < 0) ? max_deceleration_ : max_acceleration_;
      const double velocity_desired_with_vmax_amax_limits = std::min(
          {
            std::max({
              velocity_desired_[i], // Try to reach target velocity
              -max_velocity_, // Do not exceed maximum velocity
              velocity_command_[i] - acc_dec * dt, // Respect maximum acceleration / deceleration
              -sqrt(2.0 * std::max(q_[i] - lower_limit_[i] - safety_margin_, 0.0) * max_deceleration_) // Start decelerating in time before hitting limit
            }),
            max_velocity_,
            velocity_command_[i] + acc_dec * dt,
            sqrt(2.0 * std::max(upper_limit_[i] - q_[i] - safety_margin_, 0.0) * max_deceleration_)
          });

      position_command_[i] = position_command_[i] + velocity_desired_with_vmax_amax_limits * dt * gain_;
      velocity_command_[i] = velocity_desired_with_vmax_amax_limits;
    }
  } else {
    velocity_command_.setZero();
  }
  write_command();
}

void JointVelocityController::write_command() {
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

void JointVelocityController::stopping(const ros::Time& time) {
  cleanup();
  ROS_INFO("Stopping controller.");
}

void JointVelocityController::read_state() {
  for (int i = 0; i < n_joints_; i++) {
    q_(i) = joint_handles_[i].getPosition();
    qd_(i) = joint_handles_[i].getVelocity();
  }
}

void JointVelocityController::cleanup() {
  if (sim_) return;

  for (size_t i = 0; i < n_joints_; i++) {
    joint_handles_[i].setCommand(0.0);
    position_command_[i] = q_[i];
    velocity_command_[i] = 0.0;
  }
};

void JointVelocityController::joint_callback(const sensor_msgs::JointStateConstPtr& msg) {
  if (msg->velocity.size() != n_joints_) {
    ROS_ERROR_STREAM(
        "Target joint configuration message has the wrong size: " << msg->velocity.size());
    return;
  }

  Eigen::VectorXd joint_desired(n_joints_);
  for (int i = 0; i < n_joints_; i++) velocity_desired_[i] = msg->velocity[i];
  velocity_available_ = true;
}
}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::JointVelocityController, controller_interface::ControllerBase)
