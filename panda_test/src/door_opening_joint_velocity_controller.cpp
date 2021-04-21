#include <panda_test/door_opening_joint_velocity_controller.h>

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
#include <franka/exception.h>

namespace panda_test{

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
        ROS_ERROR("DoorOpeningJointVelocityController: Could not get parameter max_duration_between_commands");
        return false;
    }

    std::vector<double> temp_max_vel;
    if (!node_handle.getParam("max_velocity", temp_max_vel))
    {
        ROS_ERROR("DoorOpeningJointVelocityController: Could not get parameter max_velocity");
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
        ROS_ERROR("DoorOpeningJointVelocityController: Could not get parameter max_acceleration");
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
        ROS_ERROR("DoorOpeningJointVelocityController: Could not get parameter max_jerk");
        return false;
    }

    if (temp_max_jerk.size() != 7)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Wrong number of joint jerk limits!");
        return false;
    }

    //----- Get Franka State Interface and Handle -----

    auto state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
        ROS_ERROR("DoorOpeningJointVelocityController: Could not get state interface from hardware");
        return false;
    }

    try
    {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Exception getting state handle: " << e.what());
        return false;
    }

    //----- Get Franka Model Interface and Handle -----

    auto model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Error getting model interface from hardware");
        return false;
    }

    try
    {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException& ex)
    {
        ROS_ERROR_STREAM("DoorOpeningJointVelocityController: Exception getting model handle from interface: " << ex.what());
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

    joint_velocity_command_subscriber_ = node_handle.subscribe("/arm_command", 10, &DoorOpeningJointVelocityController::command_cb, this);

    //----- Services -----

    get_robot_state_srv = node_handle.advertiseService("/panda_state_srv", &DoorOpeningJointVelocityController::state_clb, this);
    ROS_INFO("Panda state service is ready!");

    return true;

}

//----- Starting function -----

void DoorOpeningJointVelocityController::starting(const ros::Time & /* time */)
{

    time_since_last_command_ = ros::Duration(0.0);
    desired_joint_velocity_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

}

//----- Callback function -----

void DoorOpeningJointVelocityController::command_cb(const panda_test::desired_vel_msg::ConstPtr& msg)
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
    //ROS_INFO("Time: %f", time_since_last_command_.toSec())
    if (time_since_last_command_.toSec() > max_duration_between_commands_)
    {
        desired_joint_velocity_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    robot_state_ = state_handle_->getRobotState();

    auto velocity_command = franka::limitRate(
        max_velocity_,
        max_acceleration_,
        max_jerk_,
        desired_joint_velocity_command_,
        robot_state_.dq_d,
        robot_state_.ddq_d);

    //ROS_INFO("VELOCITY COMMANDED: %f", velocity_command[6]);
    for(size_t i=0; i<7; i++)
    {
        velocity_joint_handles_[i].setCommand(velocity_command[i]);
    }

}

//----- State callback -----

bool DoorOpeningJointVelocityController::state_clb(panda_test::PandaStateSrv::Request &req, panda_test::PandaStateSrv::Response &res)
{

    try{

        auto current_state = robot_state_;

        std::array<double, 7> coriolis = model_handle_->getCoriolis();
        std::array<double, 7> gravity = model_handle_->getGravity();
        std::array<double, 42> jacobian = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        std::array<double, 49> mass_matrix = model_handle_->getMass();

        for (size_t i=0; i<7; ++i){

            res.q_d[i] = current_state.q_d[i];
            res.dq_d[i] = current_state.dq_d[i];
            res.ddq_d[i] = current_state.ddq_d[i];
            res.q[i] = current_state.q[i];
            res.dq[i] = current_state.dq[i];

            res.tau_d_no_gravity[i] = current_state.tau_J_d[i];
            res.tau[i] = current_state.tau_J[i];
            res.tau_ext[i] = current_state.tau_ext_hat_filtered[i];

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

            res.EE_T_K[i] = current_state.EE_T_K[i];
            res.O_T_EE[i] = current_state.O_T_EE[i];
        }

        for (size_t i=0; i<6; ++i){

            res.K_F_ext_hat_K[i] = current_state.K_F_ext_hat_K[i];
        }

    } catch (const franka::Exception& ex){

        ROS_ERROR_STREAM("Failed to get the robot state with error: "<<ex.what());
        return false;
    }
    return true;
}

//----- Stopping Function -----

void DoorOpeningJointVelocityController::stopping(const ros::Time & /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}


} //namespace panda_test

PLUGINLIB_EXPORT_CLASS(panda_test::DoorOpeningJointVelocityController, controller_interface::ControllerBase)

