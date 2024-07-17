#ifndef GIRAFFE_MOVEIT_H
#define GIRAFFE_MOVEIT_H

#include "moveit/robot_model/joint_model_group.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include <boost/array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>

class GiraffeMoveItUtils {
public:
  GiraffeMoveItUtils(ros::NodeHandle &nh);

  bool goToSafePos();

private:
  ros::AsyncSpinner _spinner;

  moveit::planning_interface::MoveGroupInterface _move_group_interface;
  moveit::planning_interface::PlanningSceneInterface
      _planning_scene_interface{};
  const moveit::core::JointModelGroup *_joint_model_group;
  std::string _arm_id;

  void loadParameters();
  void setup();
  void printMoveItInfo();
  bool goToJointGoal(const std::vector<double>& joints);
  bool atJoints(const std::vector<double>& joints, const double tolerance=1e-3);
};

#endif // GIRAFFE_MOVEIT_H