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

#include <moma_cartesian_impedance_controller/pseudo_inversion.hpp>

// TODO become stiffer next to joint limits
// TODO when force is above a certain threshold do not accept poses and keep the current one
// TODO (optional) get the desired pose for nullspace prjection (could come from MPC for example)

namespace moma_controllers
{

  bool CartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw,
                                          ros::NodeHandle &node_handle)
  {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    sub_equilibrium_pose_ = node_handle.subscribe(
        "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
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

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

    dynamic_server_compliance_param_ = std::make_unique<
        dynamic_reconfigure::Server<moma_cartesian_impedance_controller::compliance_paramConfig>>(

        dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    cartesian_stiffness_i_.setZero();
    windup_limit_.setZero();
    errorIntegrator_.setZero();

    desiredPosePub_ = node_handle.advertise<geometry_msgs::PoseStamped>("desired_pose", 1, false);
    desiredPoseFilteredPub_ =
        node_handle.advertise<geometry_msgs::PoseStamped>("desired_pose_filtered", 1, false);
    measuredPosePub_ = node_handle.advertise<geometry_msgs::PoseStamped>("measured_pose", 1, false);
    trackingErrorPub_ =
        node_handle.advertise<std_msgs::Float64MultiArray>("tracking_error", 1, false);
    integratorWrenchPub_ =
        node_handle.advertise<geometry_msgs::WrenchStamped>("integrator_wrench", 1, false);
    publishingTimer_ =
        node_handle.createTimer(ros::Duration(0.02), [this](const ros::TimerEvent &timerEvent)
                                {
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

        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        tf::poseEigenToMsg(transform, measuredPose.pose);

        {
          std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
          tf::pointEigenToMsg(position_d_target_, desiredPose.pose.position);
          tf::quaternionEigenToMsg(orientation_d_target_, desiredPose.pose.orientation);
          tf::pointEigenToMsg(position_d_, desiredPoseFiltered.pose.position);
          tf::quaternionEigenToMsg(orientation_d_, desiredPoseFiltered.pose.orientation);

          error.data.push_back((position_d_target_ - transform.translation()).norm());
          error.data.push_back(
              orientation_d_target_.angularDistance(Eigen::Quaterniond(transform.linear())));

          Eigen::Vector3d force = errorIntegrator_.head(3);
          Eigen::Vector3d torque = errorIntegrator_.tail(3);
          tf::vectorEigenToMsg(force, integratorWrench.wrench.force);
          tf::vectorEigenToMsg(torque, integratorWrench.wrench.torque);
        }

        desiredPosePub_.publish(desiredPose);
        desiredPoseFilteredPub_.publish(desiredPoseFiltered);
        measuredPosePub_.publish(measuredPose);
        trackingErrorPub_.publish(error);
        integratorWrenchPub_.publish(integratorWrench); });

    return true;
  }

  void CartesianImpedanceController::starting(const ros::Time & /*time*/)
  {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;

    errorIntegrator_.setZero();
  }

  void CartesianImpedanceController::update(const ros::Time & /*time*/,
                                            const ros::Duration & /*period*/)
  {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
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

    // Eigen::VectorXd signs = error.cwiseSign().cwiseProduct(errorIntegrator_.cwiseSign());
    // for (size_t i = 0; i < 6; ++i) {
    //   if (signs(i) < resetIntegratorThreshold_) {
    //     errorIntegrator_(i) = 0;
    //     std::cout << "reset i=" << i << std::endl;
    //   }
    //   else{
    //           std::cout << "keep i=" << i << std::endl;
    //   }
    // }

    errorIntegrator_ += cartesian_stiffness_i_ * error;
    errorIntegrator_ = errorIntegrator_.cwiseMin(windup_limit_);
    errorIntegrator_ = errorIntegrator_.cwiseMax(-windup_limit_);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    Eigen::VectorXd targetWrench =
        -cartesian_stiffness_ * error - errorIntegrator_ - cartesian_damping_ * (jacobian * dq);

    if (targetWrench.head(3).norm() > forceLimit_)
    {
      targetWrench.head(3) = targetWrench.head(3) / targetWrench.head(3).norm() * forceLimit_;
    }
    if (targetWrench.tail(3).norm() > torqueLimit_)
    {
      targetWrench.tail(3) = targetWrench.tail(3) / targetWrench.tail(3).norm() * torqueLimit_;
    }

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() * targetWrench;
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);
    // Desired torque
    tau_d << tau_task + tau_nullspace + coriolis;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i));
    }

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

    cartesian_stiffness_i_ = filter_params_ * cartesian_stiffness_target_i_ +
                             (1.0 - filter_params_) * cartesian_stiffness_i_;
    windup_limit_ = filter_params_ * windup_limit_target_ + (1.0 - filter_params_) * windup_limit_;

    forceLimit_ = filter_params_ * forceLimitTarget_ + (1.0 - filter_params_) * forceLimit_;
    torqueLimit_ = filter_params_ * torqueLimitTarget_ + (1.0 - filter_params_) * torqueLimit_;

    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void CartesianImpedanceController::complianceParamCallback(
      moma_cartesian_impedance_controller::compliance_paramConfig &config,
      uint32_t /*level*/)
  {
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << config.translational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << config.rotational_stiffness * Eigen::Matrix3d::Identity();

    cartesian_stiffness_target_i_.setIdentity();
    cartesian_stiffness_target_i_.topLeftCorner(3, 3)
        << config.translational_stiffness_i * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_i_.bottomRightCorner(3, 3)
        << config.rotational_stiffness_i * Eigen::Matrix3d::Identity();
    windup_limit_target_.setZero();
    windup_limit_target_.head(3) = config.translational_windup_limit * Eigen::Vector3d::Ones();
    windup_limit_target_.tail(3) = config.rotational_windup_limit * Eigen::Vector3d::Ones();

    cartesian_damping_target_.setIdentity();
    // Damping ratio = 1
    cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * sqrt(config.translational_stiffness) *
                                                         Eigen::Matrix3d::Identity() *
                                                         config.translational_damping_ratio;
    cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(config.rotational_stiffness) *
                                                             Eigen::Matrix3d::Identity() *
                                                             config.rotational_damping_ratio;
    nullspace_stiffness_target_ = config.nullspace_stiffness;
    resetIntegratorThreshold_ = config.reset_integrator_threshold;

    forceLimitTarget_ = config.max_force;
    torqueLimitTarget_ = config.max_torque;
  }

  void CartesianImpedanceController::equilibriumPoseCallback(
      const geometry_msgs::PoseStampedConstPtr &msg)
  {

    // clang-format off
    std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
    position_d_target_ << msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z;

    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z, 
                                      msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
    {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
    // clang-format on
  }

} // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::CartesianImpedanceController,
                       controller_interface::ControllerBase)
