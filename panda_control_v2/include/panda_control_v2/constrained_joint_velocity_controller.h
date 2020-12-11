#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka/rate_limiting.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include "panda_control_v2/Command.h"
#include "panda_control_v2/FullState.h"
#include "ros/ros.h"

namespace panda_control_v2 {

class ConstrainedJointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface,
					   franka_hw::FrankaModelInterface> {
public:

  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  void command_cb(const panda_control_v2::Command::ConstPtr& msg);

private:
  
  //----- Interface for joint velocity control of the hw -----

  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  
  //----- Interface for getting the robot state from hw -----

  franka_hw::FrankaStateInterface* franka_state_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;

  //----- Interface for getting the kinematic and dynamic parameters from hw -----

  franka_hw::FrankaModelInterface* model_interface_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  //----- Subscriber for joint velocity commands -----

  ros::Subscriber joint_velocity_command_subscriber_;
  std::array<double, 7> desired_joint_velocity_command_;

  //----- Publisher for robot state ------ 

  realtime_tools::RealtimePublisher<panda_control_v2::FullState> full_state_pub_;

  //----- Other parameters -----

  franka_hw::TriggerRate rate_trigger_{500.0};
  double max_duration_between_commands_;
  ros::Duration time_since_last_command_;

};

}  // namespace panda_control_v2
