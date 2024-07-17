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

#include <std_srvs/Trigger.h>

#include <ros/ros.h>

class GiraffeMoveItUtils {
public:
  GiraffeMoveItUtils(ros::NodeHandle &nh);

  bool goToSafePos();
  bool goToSafePosCB(std_srvs::Trigger::Request &_,
                     std_srvs::Trigger::Response &res);

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
  bool goToJointGoal(const std::vector<double> &joints);
  bool atJoints(const std::vector<double> &joints,
                const double tolerance = 1e-3);

  ros::ServiceServer _srv;
};

#endif // GIRAFFE_MOVEIT_H