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
#include <moma_gazebo/PandaStateSrv.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>


namespace moma_gazebo{

class DoorOpeningJointVelocityController : public controller_interface::MultiInterfaceController<
                                                    hardware_interface::VelocityJointInterface> {

public:

    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

    void command_cb(const moma_gazebo::command::ConstPtr& msg);

    bool state_clb(moma_gazebo::PandaStateSrv::Request &req, moma_gazebo::PandaStateSrv::Response &res);

    //----- Subscribers to the gazebo topics -----

    void joint_state_clb(const sensor_msgs::JointState& msg);

    void eeforce_clb(const geometry_msgs::WrenchStamped& msg);

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

    //----- Params for the state service -----

    ros::ServiceServer get_robot_state_srv;

    ros::Subscriber joint_state_subscriber;
    ros::Subscriber eeforce_subscriber;

    std::array<double, 7> q_d;
    std::array<double, 7> dq_d;
    std::array<double, 7> ddq_d;
    std::array<double, 7> q;
    std::array<double, 7> dq;
    std::array<double, 7> tau_J_d;
    std::array<double, 7> tau_J;
    std::array<double, 7> tau_ext_hat_filtered;

    std::array<double, 42> jacobian;
    std::array<double, 49> mass_matrix;
    std::array<double, 7> coriolis;
    std::array<double, 7> gravity;

    std::array<double, 16> EE_T_K;
    std::array<double, 16> O_T_EE;

    std::array<double, 6> K_F_ext_hat_K;

    bool processing = false;

};


} //namespace moma_gazebo


