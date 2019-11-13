#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import moveit_commander

import actionlib

from panda_grasp_demo.msg import DropAction, DropResult
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class DropActionServer(object):
  def __init__(self):
    super(DropActionServer, self).__init__()

    # Setup MoveIt
    self._setup_moveit()

    # Poses [x y z qx qy qz qw]
    self.drop_pose = [
      [0.307, 0.0, 5.903, 9.23879533e-01, -3.82683432e-01,  1.32497756e-12,  3.19885881e-12],
    ]

    # Set up action server
    action_name = "drop_action"
    self._as = actionlib.SimpleActionServer(action_name, DropAction, execute_cb=self.execute_cb, auto_start=False)
    self._as.start()

  def _setup_moveit(self):
    moveit_commander.roscpp_initialize(sys.argv)

    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "panda_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()
    self.group_names = self.robot.get_group_names()

    print("============ Planning frame: %s" % self.planning_frame)
    print("============ End effector link: %s" % self.eef_link)
    print("============ Available Planning Groups:", self.group_names)

    # Scale accelerations and velocities
    self.move_group.set_max_acceleration_scaling_factor(0.5)
    self.move_group.set_max_velocity_scaling_factor(0.5)

  def execute_cb(self, goal):
    rospy.loginfo("Dropping action was triggered")
    result = DropResult()

    self.go_to_pose_goal(self.scan_poses[i])

    # TODO open gripper

    rospy.loginfo("Dropping action succeeded")
    self._as.set_succeeded(result)

  def go_to_pose_goal(self, pose):
    # pose = [x y z qx qy qz qw]
    
    move_group = self.move_group

    # Pose goal
    pose_goal = Pose()
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]

    move_group.set_pose_target(pose_goal)

    # Execute
    _ = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Check if it worked
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_joint_goal(self, joint_value, wait_for_feedback=True):
    self.move_group.set_joint_value_target(joint_value)
    plan = self.move_group.plan()
    if wait_for_feedback:
      raw_input("Press Enter to continue...")
    self.move_group.execute(plan, wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()


def main():
  try:
    rospy.init_node('drop_action_node')
    dropper = DropActionServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()
