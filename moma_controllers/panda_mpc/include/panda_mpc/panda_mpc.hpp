//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <robot_control/modeling/robot_wrapper.h>

#include <moma_ocs2_ros/mpc_velocity_controller.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <geometry_msgs/TwistStamped.h>

namespace moma_controllers {

class PandaMpcController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  // explicit controller to allow for a missing hardware interface
  using BASE =
      controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                     hardware_interface::EffortJointInterface,
                                                     franka_hw::FrankaStateInterface>;
  PandaMpcController() : BASE(true){};

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool init_parameters(ros::NodeHandle& node_handle);
  bool init_common_interfaces(hardware_interface::RobotHW* robot_hw);
  bool init_franka_interfaces(hardware_interface::RobotHW* robot_hw);

  void write_command();
  void compute_torque(const ros::Duration& period);
  void read_state();

  // Saturation
  void saturate_torque_rate(const std::array<double, ocs2::mobile_manipulator::ARM_INPUT_DIM>& tau_J_d);

 private:
  bool sim_;

  // dynamic model
  std::unique_ptr<rc::RobotWrapper> robot_model_;
  std::array<control_toolbox::Pid, ocs2::mobile_manipulator::ARM_INPUT_DIM> pid_controllers_;
  MpcController::state_vector_t position_command_;
  MpcController::input_vector_t velocity_command_;
  MpcController::state_vector_t position_error_;
  MpcController::input_vector_t velocity_error_;
  MpcController::joint_vector_t arm_gravity_and_coriolis_;
  MpcController::joint_vector_t arm_tau_;

  // Keep state dynamic vector to account for eventual gripper case
  Eigen::VectorXd position_current_;
  Eigen::VectorXd velocity_current_;
  Eigen::VectorXd position_integral_;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax = 1.0;

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::array<double, ocs2::mobile_manipulator::ARM_INPUT_DIM> coriolis_;
  double coriolis_factor_ = 1.0;
  franka::RobotState robot_state_;

  bool started_;
  std::unique_ptr<moma_controllers::MpcController> mpc_controller_;
};

}  // namespace moma_controllers