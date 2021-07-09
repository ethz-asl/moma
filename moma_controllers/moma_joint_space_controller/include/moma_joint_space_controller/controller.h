#pragma once
#include <robot_control/modeling/robot_wrapper.h>
#include <control_toolbox/pid.h>

#include <actionlib/server/simple_action_server.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>

#include <moma_msgs/JointAction.h>
#include "moma_joint_space_controller/trajectory_generator.h"

#include <mutex>

namespace moma_controllers {

class JointSpaceController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::JointStateInterface,
          hardware_interface::VelocityJointInterface,
          hardware_interface::EffortJointInterface> {
 public:
  using BASE =
      controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                           hardware_interface::VelocityJointInterface,
                                           hardware_interface::EffortJointInterface>;
  using ActionServer =
      actionlib::SimpleActionServer<moma_msgs::JointAction>;

  // not all interfaces are mandatory
  JointSpaceController() : BASE(true){};
  ~JointSpaceController() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void stopping(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  bool add_command_handles(hardware_interface::RobotHW* hw);
  void read_state();
  void write_command();
  void cleanup();

  void compute_profile(const Eigen::VectorXd& goal);
  void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
  void execute_callback(const moma_msgs::JointGoalConstPtr& goal);

 protected:
  bool sim_;
  int n_joints_;
  std::string robot_description_;
  std::vector<std::string> joint_names_;

  std::mutex generator_mutex_;
  std::unique_ptr<TrajectoryGenerator> generator_;

  std::atomic_bool trajectory_available_;
  ros::Subscriber trajectory_subscriber_;

  double gain_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;

  Eigen::VectorXd joint_desired_;
  Eigen::VectorXd joint_current_;

  Eigen::VectorXd position_command_;
  Eigen::VectorXd velocity_command_;

  double tolerance_;
  double max_velocity_;
  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;

  std::atomic_bool success_ = false;
  std::unique_ptr<ActionServer> action_server_;

  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  std::unique_ptr<rc::RobotWrapper> model_;
  std::vector<control_toolbox::Pid> pid_controllers_;

};
}  // namespace moma_controllers
