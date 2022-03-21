// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <robot_control/modeling/robot_wrapper.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <moma_cartesian_impedance_controller/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace moma_controllers
{
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

struct CartesianImpedanceParams
{
  double forceLimit_;
  double torqueLimit_;
  double nullspace_stiffness_{ 20.0 };
  double resetIntegratorThreshold_{ 2.0 };
  Matrix6d cartesian_stiffness_;
  Matrix6d cartesian_stiffness_i_;
  Vector6d windup_limit_;
  Matrix6d cartesian_damping_;

  CartesianImpedanceParams()
  {
    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    cartesian_stiffness_i_.setZero();
    windup_limit_.setZero();
  }

  template <typename T>
  void blend(T& old_value, const T& new_value, double alpha)
  {
    old_value = alpha * new_value + (1 - alpha) * old_value;
  }

  void blend(const CartesianImpedanceParams& new_params, const double alpha)
  {
    blend(cartesian_stiffness_, new_params.cartesian_stiffness_, alpha);
    blend(cartesian_damping_, new_params.cartesian_damping_, alpha);
    blend(nullspace_stiffness_, new_params.nullspace_stiffness_, alpha);
    blend(cartesian_stiffness_i_, new_params.cartesian_stiffness_i_, alpha);
    blend(windup_limit_, new_params.windup_limit_, alpha);
    blend(forceLimit_, new_params.forceLimit_, alpha);
    blend(torqueLimit_, new_params.torqueLimit_, alpha);
    resetIntegratorThreshold_ = new_params.resetIntegratorThreshold_;
  }
};

class CartesianImpedanceController
  : public controller_interface::MultiInterfaceController<
        franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
{
public:
  // not all interfaces are mandatory
  // clang-format off
  using BASE = controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
                                                              hardware_interface::EffortJointInterface, 
                                                              franka_hw::FrankaStateInterface>;
  // clang-format on
  CartesianImpedanceController() : BASE(true){};

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

private:
  bool init_params(ros::NodeHandle&);
  bool init_model(ros::NodeHandle&);
  bool init_hardware_interfaces(hardware_interface::RobotHW*);
  bool init_common_interfaces(hardware_interface::RobotHW*);
  void init_compliance_param_server(ros::NodeHandle&);
  void init_ros_sub_pub(ros::NodeHandle&);
  void init_reference();

  void read_state();

  // Saturation
  Vector7d saturateTorqueRate(const Vector7d& tau_d_calculated,
                              const Vector7d& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  CartesianImpedanceParams params_;
  CartesianImpedanceParams new_params_;

  bool initialized_ = false;
  double filter_params_{0.005};
  bool sim_{ false };
  std::string arm_id_;
  std::string arm_description_;
  std::vector<std::string> joint_names_;
  const std::string ee_frame_id_ = "panda_EE";
  const std::string base_frame_id_ = "panda_link0";
  std::string target_frame_id_ = "panda_EE";
  const double delta_tau_max_{ 1.0 };

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Vector7d q_d_nullspace_;


  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Vector6d error_integrator_;

  // Model for simulation
  Eigen::VectorXd q_;  // they might contain the finger in
  Eigen::VectorXd qd_;
  Vector7d limits_gains_;
  Vector7d tau_;
  Vector7d q_min_;
  Vector7d q_max_;
  double safety_margin_;
  Eigen::Affine3d T_tool_ee_;
  std::unique_ptr<rc::RobotWrapper> model_;

  // Dynamic reconfigure
  using compliance_param_cfg = moma_cartesian_impedance_controller::compliance_paramConfig;
  std::unique_ptr<dynamic_reconfigure::Server<compliance_param_cfg>> dynamic_server_param_;

  ros::NodeHandle dynamic_reconfigure_node_;
  void complianceParamCallback(compliance_param_cfg& config, uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Publishers
  ros::Timer publishingTimer_;
  ros::Publisher desiredPosePub_;
  ros::Publisher desiredPoseFilteredPub_;
  ros::Publisher measuredPosePub_;
  ros::Publisher trackingErrorPub_;
  ros::Publisher integratorWrenchPub_;
};

}  // namespace moma_controllers

std::ostream& operator<<(std::ostream& os, const moma_controllers::CartesianImpedanceParams& params);
