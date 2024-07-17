#include <moma_utils_cpp/giraffe_moveit.h>

GiraffeMoveItUtils::GiraffeMoveItUtils(ros::NodeHandle &nh)
    : _spinner(1), _move_group_interface("panda_arm") {
  _spinner.start();
  loadParameters();

  _move_group_interface =
      moveit::planning_interface::MoveGroupInterface(_arm_id);

  ROS_INFO("Initializing MoveGroupInterface for planning group: %s",
           _arm_id.c_str());
  _joint_model_group =
      _move_group_interface.getCurrentState()->getJointModelGroup(_arm_id);

  _srv = nh.advertiseService("go_to_safe_pos",
                             &GiraffeMoveItUtils::goToSafePosCB, this);
}

void GiraffeMoveItUtils::loadParameters() {
  ros::param::param<std::string>("~arm_id", _arm_id, "panda_arm");
}

void GiraffeMoveItUtils::printMoveItInfo() {
  ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                 _move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 _move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(_move_group_interface.getJointModelGroupNames().begin(),
            _move_group_interface.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
}

bool GiraffeMoveItUtils::atJoints(const std::vector<double> &joints,
                                  const double tolerance) {
  std::vector<double> current_joints;
  _move_group_interface.getCurrentState()->copyJointGroupPositions(
      _joint_model_group, current_joints);

  // compare sizes for requested joints and current
  for (size_t i = 0; i < joints.size(); ++i) {
    ROS_INFO("joint %ld : %.3f", i, fabs(current_joints[i] - joints[i]));
    if (fabs(current_joints[i] - joints[i]) > tolerance) {
      ROS_WARN("Current joint positions don't match goal");
      return false; // joint values  don't match
    }
  }
  ROS_INFO("Current joint positions match goal");
  return true; // joint values match
}

bool GiraffeMoveItUtils::goToJointGoal(const std::vector<double> &joints) {
  ROS_INFO("Going to these joint goal positions:");
  for (size_t i = 0; i < joints.size(); ++i) {
    ROS_INFO("Joint %zu: %.3f", i, joints[i]);
  }

  _move_group_interface.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan next_plan;
  ROS_INFO("planning move");
  // HERE srv call gets stuck, suspected threading issue
  bool success = (_move_group_interface.plan(next_plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("now attempting move");
    _move_group_interface.move();
  } else {
    ROS_WARN("Planning to go to goal failed");
  }

  return success;
}

bool GiraffeMoveItUtils::goToSafePos() {
  std::vector<double> safe_joints = {0.087, -0.873, -0.070, -2.09,
                                     0.017, 1.326,  0.838};
  if (goToJointGoal(safe_joints)) {
    if (atJoints(safe_joints)) {
      ROS_INFO("Joints all at safe joint positions");
      return true;
    } else {
      ROS_WARN("Joints not at safe pos");
    }
  }
  return false;
}

bool GiraffeMoveItUtils::goToSafePosCB(std_srvs::Trigger::Request &_,
                                       std_srvs::Trigger::Response &res) {
  bool success = goToSafePos();
  ros::Duration(2.0).sleep();
  res.success = success;
  res.message =
      success ? "Moved panda to safe pos" : "Failed to move panda to safe pos";
  return success;
}