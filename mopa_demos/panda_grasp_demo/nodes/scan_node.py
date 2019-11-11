#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from moveit_commander.conversions import pose_to_list
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

import tf
import actionlib
from panda_grasp_demo.msg import ScanSceneAction, ScanSceneFeedback, ScanSceneResult

import copy
import numpy as np
import pickle


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

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self, with_as=False):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    # ROS init
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pointcloud_scan_action')

    # Subscribe to pointcloud topic
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud_callback)
    self.pointcloud_data = None

    # Poses [x y z qx qy qz qw]
    # self.scan_poses = [
    #   [0.609, 0.013, 0.552, 0.865, -0.318, -0.344, 0.182],
    #   [0.159, -0.015, 0.571, 0.339, 0.845, 0.139, 0.390]
    # ]
    self.scan_joints = [
      [-0.643, -0.678, 1.069, -1.823, 0.0436, 1.349, 1.104],
      [0.525, -0.684, -0.851, -2.067, 0.210, 1.460, 0.600]
    ]

    # Set up action server
    if with_as:
      self.action_name = rospy.get_name()
      self._as = actionlib.SimpleActionServer(self.action_name, ScanSceneAction, execute_cb=self.execute_cb, auto_start=False)
      # create messages that are used to publish feedback/result
      self._feedback = ScanSceneFeedback()
      self._result = ScanSceneResult()

    # Configure Moveit commander
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "panda_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    # Scale accelerations and velocities
    self.move_group.set_max_acceleration_scaling_factor(0.5)
    self.move_group.set_max_velocity_scaling_factor(0.5)

    # Add collision box for the table
    self.box_name = "table"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.65
    box_pose.pose.position.z = 0.1025
    box_size = [1.0, 1.0, 0.205]
    self.scene.add_box(self.box_name, box_pose, size=box_size)
    self.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    self.planning_frame = self.move_group.get_planning_frame()
    print "============ Planning frame: %s" % self.planning_frame
    self.eef_link = self.move_group.get_end_effector_link()
    print "============ End effector link: %s" % self.eef_link
    group_names = self.robot.get_group_names()
    print "============ Available Planning Groups:", self.robot.get_group_names()
    # print "============ Printing robot state"
    # print robot.get_current_state()
    print ""

    # Misc variables
    self.listener = tf.TransformListener()

    # Start action server
    if with_as:
      self._as.start()

  def execute_cb(self, goal):
    move_group = self.move_group

    success = True
    self._feedback.percent_complete = 0.0
    rospy.loginfo("Scanning action was triggered")

    if goal.num_scan_poses > len(self.scan_joints):
      self._result.success = False
      rospy.info("Invalid goal set")
      self._as.set_aborted(self._result)
      return

    for i in range(goal.num_scan_poses):
      if self._as.is_preempt_requested():
        rospy.loginfo("Got preempted")
        self._result.success = False
        self._as.set_preempted(self._result)
        return
      # self.go_to_pose_goal(self.scan_poses[i])
      move_group.go(self.scan_joints[i])
      move_group.stop()
      move_group.clear_pose_targets()

      res = self.store_pointcloud(i)
      self._feedback.percent_complete = float(i+1)/float(goal.num_scan_poses)
      self._feedback.pointcloud_read_success = res
      self._as.publish_feedback(self._feedback)
    
    # TODO run GPD to obtain grasp poses

    if success:
      # TODO publish grasp pose(s) on dedicated topic
      self._result.success = True
      rospy.loginfo("Succeed")
      self._as.set_succeeded(self._result)

  def pointcloud_callback(self, data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    self.pointcloud_data = copy.deepcopy(data)
     
  def transform_pointcloud(self, msg):
    frame = msg.header.frame_id
    translation, rotation = self.listener.lookupTransform('panda_base', frame, rospy.Time())
    transform_msg = TransformStamped()
    transform_msg.transform.translation.x = translation[0]
    transform_msg.transform.translation.y = translation[1]
    transform_msg.transform.translation.z = translation[2]
    transform_msg.transform.rotation.x = rotation[0]
    transform_msg.transform.rotation.y = rotation[1]
    transform_msg.transform.rotation.z = rotation[2]
    transform_msg.transform.rotation.w = rotation[3]
    transformed_msg = do_transform_cloud(msg, transform_msg)
    transformed_msg.header = msg.header
    transformed_msg.header.frame_id = 'panda_base'
    return transformed_msg

  def go_to_pose_goal(self, pose):
    # pose = [x y z qx qy qz qw]
    
    move_group = self.move_group

    # Pose goal
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]

    move_group.set_pose_target(pose_goal)

    # Execute
    plan = move_group.go(wait=True)
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

  def store_pointcloud(self, idx):
    if self.pointcloud_data is None:
      rospy.logwarn("Didn't receive any pointcloud data yet.")
      return False
    with open("/tmp/pointcloud"+str(idx)+".pkl", "wb") as f:
      pointcloud_msg = self.transform_pointcloud(self.pointcloud_data)
      pickle.dump(pointcloud_msg, f)
    rospy.loginfo("Stored pointcloud "+str(idx))
    return True

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

def main():
  try:
    with_action_server = False
    scanner = MoveGroupPythonIntefaceTutorial(with_action_server)

    if not with_action_server:
      print "============ Press `Enter` to go to first scanning pose ..."
      raw_input()
      # scanner.go_to_pose_goal(scanner.scan_poses[0])
      scanner.go_to_joint_goal(scanner.scan_joints[0])
      scanner.store_pointcloud(1)

      print "============ Press `Enter` to go to second scanning pose ..."
      raw_input()
      # scanner.go_to_pose_goal(scanner.scan_poses[1])
      scanner.go_to_joint_goal(scanner.scan_joints[1])
      scanner.store_pointcloud(2)
    else:
      rospy.spin()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()
