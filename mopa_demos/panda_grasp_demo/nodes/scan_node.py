#!/usr/bin/env python

from __future__ import print_function

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
from sensor_msgs.point_cloud2 import read_points, create_cloud

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
    rospy.init_node('pointcloud_scan_action')

    # Subscribe to pointcloud topic
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_cb)
    self.pointcloud_data = None

    # Setup MoveIt
    self._setup_moveit()

    # Poses [x y z qx qy qz qw]
    # self.scan_poses = [
    #   [0.609, 0.013, 0.552, 0.865, -0.318, -0.344, 0.182],
    #   [0.159, -0.015, 0.571, 0.339, 0.845, 0.139, 0.390]
    # ]
    self.scan_joints = [
      [-0.643, -0.678, 1.069, -1.823, 0.0436, 1.349, 1.104],
      [0.525, -0.684, -0.851, -2.067, 0.210, 1.460, 0.600],
    ]

    # Set up action server
    if with_as:
      self.action_name = rospy.get_name()
      self._as = actionlib.SimpleActionServer(self.action_name, ScanSceneAction, execute_cb=self.execute_cb, auto_start=False)
      
      # Create messages that are used to publish feedback/result
      self._feedback = ScanSceneFeedback()
      self._result = ScanSceneResult()

    # Create publisher for stitched point cloud
    self.stitched_point_cloud_pub = rospy.Publisher('/cloud_stitched', PointCloud2, queue_size=10)

    # Misc variables
    self.listener = tf.TransformListener()

    # Start action server
    if with_as:
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
    # TODO(mbreyer) check if this worked

  def execute_cb(self, goal):
    rospy.loginfo("Scanning action was triggered")

    success = True
    self._feedback.percent_complete = 0.0
    
    # if goal.num_scan_poses > len(self.scan_poses):
    #   self._result.success = False
    #   rospy.info("Invalid goal set")
    #   self._as.set_aborted(self._result)
    #   success = False
    #   return

    captured_clouds = []
    for i in range(len(self.scan_joints)):

      if self._as.is_preempt_requested():
        rospy.loginfo("Got preempted")
        self._as.set_preempted()
        success = False
        break
  
      # self.go_to_pose_goal(self.scan_poses[i])
      self.go_to_joint_goal(self.scan_joints[i])
      cloud = self.capture_point_cloud()
      captured_clouds.append(cloud)

      self._feedback.percent_complete = float(i+1)/float(goal.num_scan_poses)
      self._as.publish_feedback(self._feedback)
    
    if success:
      stitched_cloud = self.stitch_point_clouds(captured_clouds)
      self.stitched_point_cloud_pub.publish(stitched_cloud)
      rospy.loginfo("Stitched point clouds")      

      fname = "/tmp/stitched_cloud.pkl"
      with open(fname, "w") as f:
        pickle.dump(stitched_cloud, f)
        rospy.loginfo("Wrote stitched cloud to %s" % fname)

      rospy.loginfo("Succeed")
      self._result.success = True
      self._as.set_succeeded(self._result)

  def point_cloud_cb(self, data):
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

  def go_to_joint_goal(self, joint_value):
    self.move_group.set_joint_value_target(joint_value)
    plan = self.move_group.plan()
    raw_input("Press Enter to continue...")
    self.move_group.execute(plan, wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()

  def capture_point_cloud(self):
    if self.pointcloud_data is None:
      rospy.loginfo("Didn't receive any pointcloud data yet.")
      return
    point_cloud_msg = self.transform_pointcloud(self.pointcloud_data)
    return point_cloud_msg
    
  def stitch_point_clouds(self, clouds):
    assert len(clouds) == 2
  
    points_out = []
    for cloud in clouds:
      points_out += list(read_points(cloud))

    return create_cloud(cloud.header, cloud.fields, points_out)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = self.box_name in self.scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
  
    return False

def main():
  try:
    with_action_server = True
    scanner = MoveGroupPythonIntefaceTutorial(with_action_server)

    if not with_action_server:
      print("============ Press `Enter` to go to first scanning pose ...")
      raw_input()
      # scanner.go_to_pose_goal(scanner.scan_poses[0])
      scanner.go_to_joint_goal(scanner.scan_joints[0])
      # scanner.store_pointcloud(1)

      print("============ Press `Enter` to go to second scanning pose ...")
      raw_input()
      # scanner.go_to_pose_goal(scanner.scan_poses[1])
      scanner.go_to_joint_goal(scanner.scan_joints[1])
      # scanner.store_pointcloud(2)
    else:
      rospy.spin()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()
