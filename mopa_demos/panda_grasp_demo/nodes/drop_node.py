#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import moveit_commander

import actionlib

from panda_grasp_demo.msg import DropAction, DropResult
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from moveit_commander.conversions import pose_to_list

from panda_grasp_demo.panda_commander import PandaCommander


class DropActionServer(object):
  def __init__(self):
    super(DropActionServer, self).__init__()

    # Panda commander
    self.panda_commander = PandaCommander("panda_arm")

    # Poses [x y z qx qy qz qw]
    self.drop_pose = [0.267, -0.379, 0.454, 0.937, -0.349, 0.004, -0.006]

    # Set up action server
    action_name = "drop_action"
    self._as = actionlib.SimpleActionServer(action_name, DropAction, execute_cb=self.execute_cb, auto_start=False)
    self._as.start()

  def execute_cb(self, goal):
    rospy.loginfo("Dropping action was triggered")
    result = DropResult()

    # self.go_to_pose_goal(self.scan_poses[i])
    self.panda_commander.goto_pose_target(self.drop_pose)

    rospy.sleep(1.0)

    self.panda_commander.move_gripper(width=0.07)

    rospy.loginfo("Dropping action succeeded")
    self._as.set_succeeded(result)


def main():
  try:
    rospy.init_node('drop_action_node')
    dropper = DropActionServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()
