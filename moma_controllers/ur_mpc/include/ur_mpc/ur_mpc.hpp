//
// Created by julian on 20.12.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <robot_control/modeling/robot_wrapper.h>

#include <moma_ocs2_ros/mpc_velocity_controller.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace moma_controllers {

// References:
// panda_mpc
// https://github.com/ros-controls/ros_controllers/blob/noetic-devel/velocity_controllers/include/velocity_controllers/joint_group_velocity_controller.h
// https://github.com/ros-controls/ros_controllers/blob/noetic-devel/forward_command_controller/include/forward_command_controller/forward_joint_group_command_controller.h

class UrMpcController
    : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
 public:
  UrMpcController() : started_(false) {};

  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool init_parameters(ros::NodeHandle& nh);
  bool init_common_interfaces(hardware_interface::VelocityJointInterface* hw);

  void write_command();
  void compute_command(const ros::Duration& period);
  void read_state();

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

 private:
  ros::Time last_start_time_;

  std::string odom_topic_;
  std::string command_base_topic_;
  ros::Subscriber odom_sub_;
  ros::Publisher command_base_pub_;
  geometry_msgs::Twist base_velocity_command_;

  // dynamic model
  std::array<control_toolbox::Pid, ocs2::mobile_manipulator::ARM_INPUT_DIM> arm_pid_controllers_;
  MpcController::state_vector_t position_command_;
  MpcController::input_vector_t velocity_command_;
  MpcController::state_vector_t position_error_;
  MpcController::input_vector_t velocity_error_;
  realtime_tools::RealtimeBuffer<std::vector<double>> arm_vel_buffer_;

  // Keep state dynamic vector to account for eventual gripper case
  Eigen::VectorXd position_current_;
  Eigen::VectorXd velocity_current_;
  Eigen::VectorXd position_integral_;

  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<std::string> joint_names_;
  double measurement_trust_factor_ = 0.99;

  bool started_;
  std::unique_ptr<moma_controllers::MpcController> mpc_controller_;
};

}  // namespace moma_controllers