#include <moma_gazebo/gazebo_door_opening_joint_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>

namespace moma_gazebo{

bool DoorOpeningJointVelocityController::init(hardware_interface::RobotHW *robot_hw,
                                       ros::NodeHandle &node_handle)
{

    //----- Get Arm ID -----

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
        ROS_ERROR("DoorOpeningJointVelocityController: Could not get parameter arm_id");
        return false;
    }

    //----- Get Joint names -----

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names))
    {
        ROS_ERROR("DoorOpeningJointVelocityController: Could not parse joint names");
    }

    if (joint_names.size() != 7)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Wrong number of joint names!");
        return false;
    }

    //----- Get Hardware Limit Params -----


    if (!node_handle.getParam("max_duration_between_commands", max_duration_between_commands_))
    {
        ROS_ERROR("ConstrainedJointVelocityController: Could not get parameter max_duration_between_commands");
        return false;
    }

    std::vector<double> temp_max_vel;
    if (!node_handle.getParam("max_velocity", temp_max_vel))
    {
        ROS_ERROR("ConstrainedJointVelocityController: Could not get parameter max_velocity");
        return false;
    }

    if (temp_max_vel.size() != 7)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Wrong number of joint velocity limits!");
        return false;
    }

    std::vector<double> temp_max_acc;
    if (!node_handle.getParam("max_acceleration", temp_max_acc))
    {
        ROS_ERROR("ConstrainedJointVelocityController: Could not get parameter max_acceleration");
        return false;
    }

    if (temp_max_acc.size() != 7)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Wrong number of joint acceleration limits!");
        return false;
    }

    std::vector<double> temp_max_jerk;
    if (!node_handle.getParam("max_jerk", temp_max_jerk))
    {
        ROS_ERROR("ConstrainedJointVelocityController: Could not get parameter max_jerk");
        return false;
    }

    if (temp_max_jerk.size() != 7)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Wrong number of joint jerk limits!");
        return false;
    }

    //----- Get Velocity Joint Interface -----

    velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr)
    {
        ROS_ERROR("DoorOpeningJointVelocityController: Error getting velocity joint interface from hardware!");
        return false;
    }

    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i)
    {
        try
        {
            velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            max_velocity_[i] = temp_max_vel[i];
            max_acceleration_[i] = temp_max_acc[i];
            max_jerk_[i] = temp_max_jerk[i];

        } catch (const hardware_interface::HardwareInterfaceException& ex)
        {
            ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    //----- Subscribers -----

    joint_velocity_command_subscriber_ = node_handle.subscribe("/arm_joint_command", 10, &DoorOpeningJointVelocityController::command_cb, this);

    joint_state_subscriber = node_handle.subscribe("/joint_states", 10, &DoorOpeningJointVelocityController::joint_state_clb, this);
    eeforce_subscriber = node_handle.subscribe("/eeforce", 10, &DoorOpeningJointVelocityController::eeforce_clb, this);

    //----- Service -----

    get_robot_state_srv = node_handle.advertiseService("/get_panda_state_srv", &DoorOpeningJointVelocityController::state_clb, this);
    ROS_INFO("Panda state service is ready!");

    for (size_t i=0; i<42; ++i){
        jacobian[i] = i;
    }

    for (size_t i=0; i<49; ++i){
        mass_matrix[i] = i;
    }

    for (size_t i=0; i<7; ++i){
        q_d[i] = i;
        dq_d[i] = i;
        ddq_d[i] = i;
        q[i] = i;
        dq[i] = i;
        tau_J[i] = i;
        tau_J_d[i] = i;
        tau_ext_hat_filtered[i] = i;
        coriolis[i] = i;
        gravity[i] = i;
    }
    EE_T_K = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 1};
    O_T_EE = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.3, 0.2, 0.5, 1};
    K_F_ext_hat_K = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    return true;

}

//----- Starting function -----

void DoorOpeningJointVelocityController::starting(const ros::Time & /* time */)
{

    time_since_last_command_ = ros::Duration(0.0);
    desired_joint_velocity_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

}

//----- Callback function -----

void DoorOpeningJointVelocityController::command_cb(const moma_gazebo::command::ConstPtr& msg)
{

    desired_joint_velocity_command_[0] = msg->dq_arm[0];
    desired_joint_velocity_command_[1] = msg->dq_arm[1];
    desired_joint_velocity_command_[2] = msg->dq_arm[2];
    desired_joint_velocity_command_[3] = msg->dq_arm[3];
    desired_joint_velocity_command_[4] = msg->dq_arm[4];
    desired_joint_velocity_command_[5] = msg->dq_arm[5];
    desired_joint_velocity_command_[6] = msg->dq_arm[6];

    time_since_last_command_ = ros::Duration(0.0);

}

//----- Update Function -----

void DoorOpeningJointVelocityController::update(const ros::Time & /* time */, const ros::Duration &period)
{

    time_since_last_command_ += period;
    if (time_since_last_command_.toSec() > max_duration_between_commands_)
    {
        desired_joint_velocity_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

//    auto state = state_handle_->getRobotState();
//
//    auto velocity_command = franka::limitRate(
//        max_velocity_,
//        max_acceleration_,
//        max_jerk_,
//        desired_joint_velocity_command_,
//        state.dq_d,
//        state.ddq_d);


    for(size_t i=0; i<7; i++)
    {
        velocity_joint_handles_[i].setCommand(desired_joint_velocity_command_[i]);
    }

}

//----- Service callback ------

bool DoorOpeningJointVelocityController::state_clb(moma_gazebo::PandaStateSrv::Request &req, moma_gazebo::PandaStateSrv::Response &res){

    processing = true;
    try{

        for (size_t i=0; i<7; ++i){

            res.q_d[i] = q_d[i];
            res.dq_d[i] = dq_d[i];
            res.ddq_d[i] = ddq_d[i];
            res.q[i] = q[i];
            res.dq[i] = dq[i];

            res.tau_d_no_gravity[i] = tau_J_d[i];
            res.tau[i] = tau_J[i];
            res.tau_ext[i] = tau_ext_hat_filtered[i];

            res.coriolis[i] = coriolis[i];
            res.gravity[i] = gravity[i];
        }

        for (size_t i=0; i<42; ++i){

            res.jacobian[i] = jacobian[i];
        }

        for (size_t i=0; i<49; ++i){

            res.mass_matrix[i] = mass_matrix[i];
        }

        for (size_t i=0; i<16; ++i){

            res.EE_T_K[i] = EE_T_K[i];
            res.O_T_EE[i] = O_T_EE[i];
        }

        for (size_t i=0; i<6; ++i){

            res.K_F_ext_hat_K[i] = K_F_ext_hat_K[i];
            }

    } catch (const std::exception& e){
        processing = false;
        ROS_ERROR_STREAM("Failed to get the robot state with error");
        return false;
    }
    processing = false;
    return true;
}

//----- Joint state callback function -----

void DoorOpeningJointVelocityController::joint_state_clb(const sensor_msgs::JointState& msg) {

    if (~processing){

        if (msg.position.size() == 12){
            for (size_t i=3; i<10; ++i){
                q_d[i - 3] = msg.position[i];
                dq_d[i -3] = msg.velocity[i];
                tau_J[i - 3] = msg.effort[i];
            }
        }
    }
}

//----- Force callback function -----

void DoorOpeningJointVelocityController::eeforce_clb(const geometry_msgs::WrenchStamped& msg){

    if (~processing){
        K_F_ext_hat_K[0] = msg.wrench.force.x;
        K_F_ext_hat_K[1] = msg.wrench.force.y;
        K_F_ext_hat_K[2] = msg.wrench.force.z;

        K_F_ext_hat_K[3] = msg.wrench.torque.x;
        K_F_ext_hat_K[4] = msg.wrench.torque.y;
        K_F_ext_hat_K[5] = msg.wrench.torque.z;
    }
}

//----- Stopping Function -----

void DoorOpeningJointVelocityController::stopping(const ros::Time & /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}


} //namespace moma_gazebo

PLUGINLIB_EXPORT_CLASS(moma_gazebo::DoorOpeningJointVelocityController, controller_interface::ControllerBase)       //this moma_gazebo corresponds to the namespace

