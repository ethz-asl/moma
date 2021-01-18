#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <moma_gazebo/command.h>


namespace moma_gazebo{

class DoorOpeningJointVelocityController : public controller_interface::MultiInterfaceController<
                                                    hardware_interface::VelocityJointInterface> {

public:

    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

    void command_cb(const moma_gazebo::command::ConstPtr& msg);

private:

    //----- Interface for joint velocity control of the hw -----

    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

    //----- Subscriber for joint velocity commands -----

    ros::Subscriber joint_velocity_command_subscriber_;
    std::array<double, 7> desired_joint_velocity_command_;

    //----- Other parameters -----

    ros::Duration time_since_last_command_;

    double max_duration_between_commands_;

    std::array<double, 7> max_velocity_;
    std::array<double, 7> max_acceleration_;
    std::array<double, 7> max_jerk_;

};


} //namespace moma_gazebo


