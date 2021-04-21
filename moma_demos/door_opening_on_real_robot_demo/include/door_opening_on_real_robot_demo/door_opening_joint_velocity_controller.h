#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka/rate_limiting.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/ros.h>

#include <door_opening_on_real_robot_demo/desired_vel_msg.h>
#include <door_opening_on_real_robot_demo/PandaStateSrv.h>

namespace door_opening_on_real_robot_demo{

class DoorOpeningJointVelocityController : public controller_interface::MultiInterfaceController<
                                                    hardware_interface::VelocityJointInterface,
                                                    franka_hw::FrankaStateInterface,
                                                    franka_hw::FrankaModelInterface> {

public:

    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

    void command_cb(const door_opening_on_real_robot_demo::desired_vel_msg::ConstPtr& msg);

    bool state_clb(door_opening_on_real_robot_demo::PandaStateSrv::Request &req, door_opening_on_real_robot_demo::PandaStateSrv::Response &res);

private:

    franka::RobotState robot_state_;

    //----- Interface for joint velocity control of the hw -----

    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

    //----- Franka State handle used for rate limiting -----

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

    //----- Franka Model handle -----

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

    //----- Subscriber for joint velocity commands -----

    ros::Subscriber joint_velocity_command_subscriber_;
    std::array<double, 7> desired_joint_velocity_command_;

    //----- Other parameters -----

    ros::Duration time_since_last_command_;

    double max_duration_between_commands_;

    std::array<double, 7> max_velocity_;
    std::array<double, 7> max_acceleration_;
    std::array<double, 7> max_jerk_;

    //----- State service -----

    ros::ServiceServer get_robot_state_srv;


};


} //namespace door_opening_on_real_robot_demo


