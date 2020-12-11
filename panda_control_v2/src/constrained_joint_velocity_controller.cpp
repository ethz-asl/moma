#include <panda_control_v2/constrained_joint_velocity_controller.h>

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

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace panda_control_v2
{

bool ConstrainedJointVelocityController::init(hardware_interface::RobotHW *robot_hw,
                                       ros::NodeHandle &node_handle)
{

  //----- Get arm_id ----- 

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ConstrainedJointVelocityController: Could not read parameter arm_id");
    return false;
  }
  
  //----- Get Franke state interface -----

  franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("ConstrainedJointVelocityController: Could not get Franka state interface from hardware");
    return false;
  }

  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ConstrainedJointVelocityController: Exception getting franka state handle: " << ex.what());
    return false;
  }
 
  //----- Get Model interface -----

  model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM("ConstrainedJointVelocityController: Error getting model interface from hardware");
    return false;
  }    

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface_->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ConstrainedJointVelocityController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  //----- Get Joint Velocity interfaces -----

  velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("ConstrainedJointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }  

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("ConstrainedJointVelocityController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("ConstrainedJointVelocityController: Wrong number of joint names!");
    return false;
  }  

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ConstrainedJointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  //----- Load parameters -----

  if (!node_handle.getParam("max_duration_between_commands", max_duration_between_commands_))
  {
    ROS_ERROR("ConstrainedJointVelocityController: Could not get parameter max_duration_between_commands");
    return false;
  }

  //----- Init Publisher and Subscriber -----

  full_state_pub_.init(node_handle, "full_arm_state", 1);

  joint_velocity_command_subscriber_ = node_handle.subscribe("set_command",
                                                       10,
                                                       &ConstrainedJointVelocityController::command_cb,
                                                       this);

  return true;
}

//----- Starting Function -----
void ConstrainedJointVelocityController::starting(const ros::Time & /* time */)
{
  time_since_last_command_ = ros::Duration(0.0);
  desired_joint_velocity_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

//----- Callback for ROS message -----
void ConstrainedJointVelocityController::command_cb(const panda_control_v2::Command::ConstPtr& msg)
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
void ConstrainedJointVelocityController::update(const ros::Time & /* time */,
                                         const ros::Duration &period)
{
  time_since_last_command_ += period;
  if (time_since_last_command_.toSec() > max_duration_between_commands_)
  {
    desired_joint_velocity_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  //----- Issue a command to each of joints -----

  for (size_t i = 0; i < 7; i++) {
    velocity_joint_handles_[i].setCommand(desired_joint_velocity_command_[i]);
  }

  //----- Publish Full Robot State -----

  if (rate_trigger_() && full_state_pub_.trylock()){

    auto state = franka_state_handle_->getRobotState(); 
   
    std::array<double, 49> mass = model_handle_->getMass();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    std::array<double, 7> gravity = model_handle_->getGravity();
    std::array<double, 42> endeffector_zero_jacobian = model_handle_->getZeroJacobian(franka::Frame::kEndEffector); 

    //----- Fill all the messages of length 7 -----

    for (size_t i = 0; i < state.q.size(); i++) {
      full_state_pub_.msg_.Coriolis[i] = coriolis[i];
      full_state_pub_.msg_.Gravity[i] = gravity[i];
      full_state_pub_.msg_.q[i] = state.q[i];
      full_state_pub_.msg_.dq[i] = state.dq[i];
      full_state_pub_.msg_.tau_J[i] = state.tau_J[i];
    }  

    //----- Fill all the messages of length 16 -----

    for (size_t i = 0; i < state.EE_T_K.size(); i++) {
      full_state_pub_.msg_.O_T_EE[i] = state.O_T_EE[i];
      full_state_pub_.msg_.F_T_EE[i] = state.F_T_EE[i];
      full_state_pub_.msg_.NE_T_EE[i] = state.NE_T_EE[i];
      full_state_pub_.msg_.EE_T_K[i] = state.EE_T_K[i];
    }

    //----- Fill all the messages of length 6 -----

    for (size_t i = 0; i < state.O_F_ext_hat_K.size(); i++) {
      full_state_pub_.msg_.O_F_ext_hat_K[i] = state.O_F_ext_hat_K[i];
      full_state_pub_.msg_.K_F_ext_hat_K[i] = state.K_F_ext_hat_K[i];
    }

    //----- Fill the mass matrix message -----

    for (size_t i = 0; i < mass.size(); i++) {
      full_state_pub_.msg_.MassMatrix[i] = mass[i];
    }

    //----- Fill the jacobian message -----

    for (size_t i = 0; i < endeffector_zero_jacobian.size(); i++) {
      full_state_pub_.msg_.Jacobian[i] = endeffector_zero_jacobian[i];
    }
    
    full_state_pub_.unlockAndPublish();     
  }
  
}

//----- Stopping Function -----
void ConstrainedJointVelocityController::stopping(const ros::Time & /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}


} // namespace panda_control_v2

PLUGINLIB_EXPORT_CLASS(panda_control_v2::ConstrainedJointVelocityController,
                       controller_interface::ControllerBase)
