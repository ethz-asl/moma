#ifndef GIRAFFE_MOVEIT_H
#define GIRAFFE_MOVEIT_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class GiraffeMoveItUtils {
public:
  GiraffeMoveItUtils(ros::NodeHandle &nh);
  ~GiraffeMoveItUtils();

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
  ros::ServiceServer _srv;
  std::mutex _mutex; // to protect _move_group_interface

  std::queue<std::vector<double>> _planning_queue;
  std::thread _planning_thread;
  std::mutex _queue_mutex; // to protect the srv queue
  std::condition_variable _cv;
  bool _success = false;
  bool _shutdown_planning_thread = false;

  void loadParameters();
  void setup();
  void printMoveItInfo();
  bool
  goToJointGoal(const std::vector<double> &joints,
                std::shared_ptr<std_srvs::Trigger::Response> res = nullptr);
  bool atJoints(const std::vector<double> &joints,
                const double tolerance = 1e-3);
  void planningThread();
};

#endif // GIRAFFE_MOVEIT_H