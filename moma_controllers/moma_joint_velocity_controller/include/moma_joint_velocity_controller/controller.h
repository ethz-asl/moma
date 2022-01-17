#pragma once
#include <robot_control/modeling/robot_wrapper.h>
#include <control_toolbox/pid.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>

#include <mutex>

namespace moma_controllers {

class JointVelocityController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::JointStateInterface,
          hardware_interface::VelocityJointInterface,
          hardware_interface::EffortJointInterface> {
 public:
  using BASE =
      controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                           hardware_interface::VelocityJointInterface,
                                           hardware_interface::EffortJointInterface>;

  // not all interfaces are mandatory
  JointVelocityController() : BASE(true){};
  ~JointVelocityController() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  bool add_command_handles(hardware_interface::RobotHW* hw);
  void read_state();
  void write_command(const ros::Duration& period);
  void cleanup();

  void joint_callback(const sensor_msgs::JointStateConstPtr& msg);

 protected:
  bool sim_;
  int n_joints_;
  std::string arm_description_;
  std::vector<std::string> joint_names_;

  std::atomic_bool velocity_available_;
  ros::Subscriber velocity_subscriber_;

  double gain_;
  double max_acceleration_;
  double max_deceleration_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;

  Eigen::VectorXd velocity_desired_;

  Eigen::VectorXd position_command_;
  Eigen::VectorXd velocity_command_;

  double max_velocity_;
  double safety_margin_;
  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;

  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  std::unique_ptr<rc::RobotWrapper> model_;
  std::vector<control_toolbox::Pid> pid_controllers_;

};
}  // namespace moma_controllers
