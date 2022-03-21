#include <moma_cartesian_impedance_controller/cartesian_impedance_controller.hpp>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <moma_cartesian_impedance_controller/pseudo_inversion.hpp>

namespace moma_controllers
{
bool CartesianImpedanceController::init_params(ros::NodeHandle& node_handle)
{
  if (!node_handle.getParam("arm_id", arm_id_))
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names_) || joint_names_.size() != 7)
  {
    ROS_ERROR(
        "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("filter_params", filter_params_))
  {
    ROS_WARN_STREAM("parameter \"filter_params\" is not set. Default to " << filter_params_);
  }

  std::vector<double> lower_limit;
  if (!node_handle.getParam("lower_limit", lower_limit) || lower_limit.size() != 7)
  {
    ROS_ERROR_STREAM("Failed to get lower_limit or invalid param");
    return false;
  }

  std::vector<double> upper_limit;
  if (!node_handle.getParam("upper_limit", upper_limit) || upper_limit.size() != 7)
  {
    ROS_ERROR_STREAM("Failed to get upper_limit or invalid param");
    return false;
  }

  if (!node_handle.getParam("safety_margin", safety_margin_) || safety_margin_ < 0)
  {
    ROS_ERROR_STREAM("Failed to get safety margin or invalid param.");
    return false;
  }

  std::vector<double> q_nullspace{};
  if (!node_handle.getParam("q_nullspace", q_nullspace) || q_nullspace.size() != 7)
  {
    ROS_ERROR_STREAM("Failed to get q nullspace or invalid param.");
    return false;
  }

  std::vector<double> limits_gains{};
  if (!node_handle.getParam("limits_gains", limits_gains) || limits_gains.size() != 7)
  {
    ROS_ERROR_STREAM("Failed to get limits_gains or invalid param.");
    return false;
  }

  if (!node_handle.getParam("target_frame", target_frame_id_) || target_frame_id_.empty())
  {
    ROS_ERROR_STREAM("Failed to get target_frame or invalid param.");
    return false;
  }

  for (size_t i{}; i < 7; i++)
  {
    q_min_(i) = lower_limit[i] + 2*safety_margin_;
    q_max_(i) = upper_limit[i] - 2*safety_margin_;
    limits_gains_(i) = limits_gains[i];
  }

  ROS_INFO("All parameters successfully initialized.");
  return true;
}

bool CartesianImpedanceController::init_model(ros::NodeHandle& node_handle)
{
  if (sim_)
  {
    if (!node_handle.getParam("/arm_description", arm_description_) || arm_description_.empty())
    {
      ROS_ERROR_STREAM("Failed to get /robot_description");
      return false;
    }

    model_ = std::make_unique<rc::RobotWrapper>();
    model_->initFromXml(arm_description_);
    q_.setZero(model_->getDof());
    qd_.setZero(model_->getDof());
  }
  else
  {
    q_.setZero(7);
    qd_.setZero(7);
  }

  ROS_INFO("Model successfully initialized.");
  return true;
}

bool CartesianImpedanceController::init_common_interfaces(hardware_interface::RobotHW* hw)
{
  auto command_interface = hw->get<hardware_interface::EffortJointInterface>();
  if (command_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Can't get EffortJointInterface");
    return false;
  }

  for (size_t i = 0; i < 7; i++)
  {
    joint_handles_.push_back(command_interface->getHandle(joint_names_[i]));
  }
  ROS_INFO("All common interfaces successfully initialized.");
  return true;
}

bool CartesianImpedanceController::init_hardware_interfaces(hardware_interface::RobotHW* hw)
{
  auto* model_interface = hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try
  {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id_ + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try
  {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id_ + "_robot"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting state handle from interface: " << ex.what());
    return false;
  }
  ROS_INFO("Hardware interfaces successfully initialized.");
  return true;
}

void CartesianImpedanceController::init_compliance_param_server(ros::NodeHandle& node_handle)
{
  dynamic_reconfigure_node_ = ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_param_node");

  dynamic_server_param_ =
      std::make_unique<dynamic_reconfigure::Server<compliance_param_cfg>>(dynamic_reconfigure_node_);

  dynamic_server_param_->setCallback(boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));
  ROS_INFO("Impedance params server successfully initialized.");
}

void CartesianImpedanceController::init_ros_sub_pub(ros::NodeHandle& node_handle)
{
  sub_equilibrium_pose_ =
      node_handle.subscribe("equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
                            ros::TransportHints().reliable().tcpNoDelay());
  desiredPosePub_ = node_handle.advertise<geometry_msgs::PoseStamped>("desired_pose", 1, false);
  desiredPoseFilteredPub_ = node_handle.advertise<geometry_msgs::PoseStamped>("desired_pose_filtered", 1, false);
  measuredPosePub_ = node_handle.advertise<geometry_msgs::PoseStamped>("measured_pose", 1, false);
  trackingErrorPub_ = node_handle.advertise<std_msgs::Float64MultiArray>("tracking_error", 1, false);
  integratorWrenchPub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("integrator_wrench", 1, false);
  publishingTimer_ = node_handle.createTimer(ros::Duration(0.02), [this](const ros::TimerEvent& timerEvent) {
    if (!initialized_) return;

    auto time = timerEvent.current_real;
    const auto frameId = base_frame_id_;

    std_msgs::Float64MultiArray error;
    geometry_msgs::PoseStamped desiredPose, desiredPoseFiltered, measuredPose;
    geometry_msgs::WrenchStamped integratorWrench;
    desiredPose.header.frame_id = frameId;
    desiredPose.header.stamp = time;
    desiredPoseFiltered.header.frame_id = frameId;
    desiredPoseFiltered.header.stamp = time;
    measuredPose.header.frame_id = frameId;
    measuredPose.header.stamp = time;
    integratorWrench.header.frame_id = frameId;
    integratorWrench.header.stamp = time;

    Eigen::Affine3d T_base_ee_;
    if (sim_)
    {
      T_base_ee_ = model_->getFramePlacement(ee_frame_id_).toHomogeneousMatrix();
    }
    else
    {
      franka::RobotState robot_state = state_handle_->getRobotState();
      T_base_ee_ = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    }
    Eigen::Affine3d T_base_tool_ = T_base_ee_ * T_tool_ee_.inverse();
    tf::poseEigenToMsg(T_base_tool_, measuredPose.pose);

    {
      std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
      Eigen::Affine3d T_base_tool_desired;
      Eigen::Affine3d T_base_tool_desired_filtered;

      T_base_tool_desired.translation() = position_d_target_;
      T_base_tool_desired.linear() = Eigen::Matrix3d(orientation_d_target_);
      T_base_tool_desired = T_base_tool_desired * T_tool_ee_.inverse();
      tf::pointEigenToMsg(T_base_tool_desired.translation(), desiredPose.pose.position);
      tf::quaternionEigenToMsg(Eigen::Quaterniond(T_base_tool_desired.linear()), desiredPose.pose.orientation);

      T_base_tool_desired_filtered.translation() = position_d_target_;
      T_base_tool_desired_filtered.linear() = Eigen::Matrix3d(orientation_d_target_);
      T_base_tool_desired_filtered = T_base_tool_desired_filtered * T_tool_ee_.inverse();
      tf::pointEigenToMsg(T_base_tool_desired_filtered.translation(), desiredPoseFiltered.pose.position);
      tf::quaternionEigenToMsg(Eigen::Quaterniond(T_base_tool_desired_filtered.linear()), desiredPoseFiltered.pose.orientation);

      error.data.push_back((T_base_tool_desired_filtered.translation() - T_base_tool_.translation()).norm());
      error.data.push_back(Eigen::Quaterniond(T_base_tool_desired_filtered.linear()).angularDistance(Eigen::Quaterniond(T_base_tool_.linear())));

      Eigen::Vector3d force = error_integrator_.head(3);
      Eigen::Vector3d torque = error_integrator_.tail(3);
      tf::vectorEigenToMsg(force, integratorWrench.wrench.force);
      tf::vectorEigenToMsg(torque, integratorWrench.wrench.torque);
    }

    desiredPosePub_.publish(desiredPose);
    desiredPoseFilteredPub_.publish(desiredPoseFiltered);
    measuredPosePub_.publish(measuredPose);
    trackingErrorPub_.publish(error);
    integratorWrenchPub_.publish(integratorWrench);
  });
  ROS_INFO("All ros subscribers and publishers successfully initialized.");
}

void CartesianImpedanceController::init_reference()
{
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  error_integrator_.setZero();
  ROS_INFO("Reference successfully initialized.");
}

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  if (!init_params(node_handle))
    return false;

  if (!init_common_interfaces(robot_hw))
    return false;

  sim_ = !init_hardware_interfaces(robot_hw);

  init_model(node_handle);

  init_compliance_param_server(node_handle);

  init_reference();

  init_ros_sub_pub(node_handle);

  // find transform from target to hand
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::TransformStamped tf_tool_ee;
  try
  {
    tf_tool_ee = buffer.lookupTransform(target_frame_id_, ee_frame_id_, ros::Time(0), ros::Duration(3.0));
    tf::transformMsgToEigen(tf_tool_ee.transform, T_tool_ee_);
    ROS_INFO_STREAM("Relative transfor end effector to tool frame is\n" << T_tool_ee_.matrix());
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN(ex.what());
    return false;
  }

  initialized_ = true;
  ROS_INFO("Controller successfully initialized.");
  return true;
}

void CartesianImpedanceController::read_state()
{
  for (int i = 0; i < 7; i++)
  {
    q_(i) = joint_handles_[i].getPosition();
    qd_(i) = joint_handles_[i].getVelocity();
    tau_(i) = joint_handles_[i].getEffort();
  }
}

void CartesianImpedanceController::starting(const ros::Time& /*time*/)
{
  // set intial target
  read_state();
  ROS_INFO_STREAM("Start joints: " << q_.transpose());
  Eigen::Affine3d initial_transform;

  if (sim_)
  {
    model_->updateState(q_, qd_);
    initial_transform = Eigen::Affine3d(model_->getFramePlacement(ee_frame_id_).toHomogeneousMatrix());
  }
  else
  {
    franka::RobotState initial_state = state_handle_->getRobotState();
    initial_transform = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    ROS_INFO_STREAM("Initial transform is:\n" << initial_transform.matrix());
  }

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  ROS_INFO_STREAM("position desired is: " << position_d_.transpose());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_.head<7>();
  error_integrator_.setZero();
}

void CartesianImpedanceController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  Eigen::Affine3d transform;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::VectorXd non_linear_terms;
  Eigen::Matrix<double, 6, 7> jacobian;

  if (sim_)
  {
    read_state();
    model_->updateState(q_, qd_);
    model_->computeAllTerms();
    non_linear_terms = model_->getNonLinearTerms().head<7>();
    jacobian = model_->getFrameJacobian(ee_frame_id_).block<6, 7>(0, 0);
    transform = model_->getFramePlacement(ee_frame_id_).toHomogeneousMatrix();
    position = transform.translation();
    orientation = transform.linear();
  }
  else
  {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    non_linear_terms = Eigen::Map<Vector7d>(coriolis_array.data());
    jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
    q_ = Eigen::Map<Vector7d>(robot_state.q.data());

    // filter qd on hardware
    const double alpha = 0.9;
    qd_ = alpha * Eigen::Map<Vector7d>(robot_state.dq.data()) + (1-alpha) * qd_;  

    tau_ = Eigen::Map<Vector7d>(robot_state.tau_J_d.data());
    transform = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    position = Eigen::Vector3d(transform.translation());
    orientation = Eigen::Quaterniond(transform.linear());
  }

  // compute error to desired pose
  // position error
  Vector6d error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
  {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  //ROS_INFO_STREAM_THROTTLE(1.0, "error=" << error.transpose());

  error_integrator_ += params_.cartesian_stiffness_i_ * error;
  error_integrator_ = error_integrator_.cwiseMin(params_.windup_limit_);
  error_integrator_ = error_integrator_.cwiseMax(-params_.windup_limit_);

  // compute control
  // allocate variables
  Vector7d tau_task;
  Vector7d tau_nullspace;
  Vector7d tau_d;
  Vector7d tau_limits;

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Vector6d targetWrench =
      -params_.cartesian_stiffness_ * error - error_integrator_ - params_.cartesian_damping_ * (jacobian * qd_);

  if (targetWrench.head(3).norm() > params_.forceLimit_)
  {
    targetWrench.head(3) = targetWrench.head(3) / targetWrench.head(3).norm() * params_.forceLimit_;
  }
  if (targetWrench.tail(3).norm() > params_.torqueLimit_)
  {
    targetWrench.tail(3) = targetWrench.tail(3) / targetWrench.tail(3).norm() * params_.torqueLimit_;
  }

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * targetWrench;

  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (params_.nullspace_stiffness_ * (q_d_nullspace_ - q_.head<7>()) -
                        (2.0 * sqrt(params_.nullspace_stiffness_)) * qd_.head<7>());

  /* 
   * we apply limits 2 * safety_margin_ far from the actual limits
   * split the calucation in two parts:
   *  
   * a) 0 < abs(delta) < safety_margin_
   *    linearly interpolate towards 0 tau_task and tau_nullspace
   *  
   * b) safety_margin_ < abs(delta) < 2*safety_margin_
   *    set all other tau to zero and only apply tau limits
   *
   * tau limits is computed as a PD term, where D is to add virtual damping
   */
  Vector7d dq_limits_ =
      (q_max_ - q_.head<7>()).cwiseMin(0.0) + (q_min_ - q_.head<7>()).cwiseMax(0.0);

  tau_limits.setZero();
  for (size_t i{}; i < 7; i++) {
    const static double tol = 1e-4;
    const double delta = dq_limits_[i];
    const double delta_abs = std::abs(delta);
    const double alpha = (1 - delta_abs / safety_margin_);

    if (delta_abs > tol && (tau_task[i] * delta) < 0) {  // change tau only if acts against the limit
      tau_limits[i] = delta * limits_gains_[i] - qd_[i] * 2.0 * std::sqrt(limits_gains_[i]);
      if (delta_abs < safety_margin_){
        tau_task[i] *= alpha;
        tau_nullspace[i] *= alpha;
      }
      else{
        tau_task[i] = 0.0;
        tau_nullspace[i] = 0.0;
      }
    }
  }

  // Desired torque
  tau_d << tau_task + tau_nullspace + tau_limits + non_linear_terms;

  
  // Saturate torque rate to avoid discontinuities
  if (!sim_)
  {
    tau_d << saturateTorqueRate(tau_d, tau_);
  }

  // Write command
  for (size_t i = 0; i < 7; ++i)
  {
    joint_handles_[i].setCommand(tau_d[i]);
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  params_.blend(new_params_, filter_params_);

  std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d)
{  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++)
  {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::complianceParamCallback(
    moma_cartesian_impedance_controller::compliance_paramConfig& config, uint32_t /*level*/)
{
  new_params_.cartesian_stiffness_.setIdentity();
  new_params_.cartesian_stiffness_.topLeftCorner(3, 3) << config.translational_stiffness * Eigen::Matrix3d::Identity();
  new_params_.cartesian_stiffness_.bottomRightCorner(3, 3) << config.rotational_stiffness * Eigen::Matrix3d::Identity();

  new_params_.cartesian_stiffness_i_.setIdentity();
  new_params_.cartesian_stiffness_i_.topLeftCorner(3, 3)
      << config.translational_stiffness_i * Eigen::Matrix3d::Identity();
  new_params_.cartesian_stiffness_i_.bottomRightCorner(3, 3)
      << config.rotational_stiffness_i * Eigen::Matrix3d::Identity();
  new_params_.windup_limit_.setZero();
  new_params_.windup_limit_.head(3) = config.translational_windup_limit * Eigen::Vector3d::Ones();
  new_params_.windup_limit_.tail(3) = config.rotational_windup_limit * Eigen::Vector3d::Ones();

  new_params_.cartesian_damping_.setIdentity();
  // Damping ratio = 1
  new_params_.cartesian_damping_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity() * config.translational_damping_ratio;
  new_params_.cartesian_damping_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity() * config.rotational_damping_ratio;
  new_params_.nullspace_stiffness_ = config.nullspace_stiffness;
  new_params_.resetIntegratorThreshold_ = config.reset_integrator_threshold;
  new_params_.forceLimit_ = config.max_force;
  new_params_.torqueLimit_ = config.max_torque;
  ROS_INFO_STREAM("New params: \n" << new_params_);
}

void CartesianImpedanceController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  Eigen::Affine3d T_base_tool;
  tf::poseMsgToEigen(msg->pose, T_base_tool);
  Eigen::Affine3d T_base_ee = T_base_tool * T_tool_ee_;

  ROS_INFO_STREAM("Received new target:\n " << T_base_tool.matrix());
  ROS_INFO_STREAM("Actual target is:\n " << T_base_ee.matrix());

  // clang-format off
  std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_target_ << T_base_ee.translation();

  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << Eigen::Quaterniond(T_base_ee.linear()).coeffs();
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
  {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
  // clang-format on
}

}  // namespace moma_controllers

std::ostream& operator<<(std::ostream& os, const moma_controllers::CartesianImpedanceParams& params)
{
  os << "Cartesian stiffness" << params.cartesian_stiffness_.diagonal().transpose() << std::endl;
  os << "Cartesian damping: " << params.cartesian_damping_.diagonal().transpose() << std::endl;
  os << "Cartesian integral " << params.cartesian_stiffness_i_.diagonal().transpose() << std::endl;
  os << "Windup limit: " << params.windup_limit_.transpose() << std::endl;
  os << "reset integrator threshold: " << params.resetIntegratorThreshold_ << std::endl;
  os << "nullspace stiffness: " << params.nullspace_stiffness_ << std::endl;
  return os;
}

PLUGINLIB_EXPORT_CLASS(moma_controllers::CartesianImpedanceController, controller_interface::ControllerBase)
