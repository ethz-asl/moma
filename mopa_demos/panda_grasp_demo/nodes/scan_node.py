#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import moveit_commander
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from moveit_commander.conversions import pose_to_list
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from sensor_msgs.point_cloud2 import read_points, create_cloud
from gpd_ros.msg import GraspConfigList
import tf
import actionlib
from panda_grasp_demo.msg import ScanSceneAction, ScanSceneFeedback, ScanSceneResult
import copy
import numpy as np
import pickle
from scipy.spatial.transform import Rotation


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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self, with_as=False):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    # ROS init
    rospy.init_node('scan_action_node')

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
      [1.1617, -0.5194, -1.6787, -1.345415587151993, 0.27857345296034125, 0.9858770641287168, 0.48917187201250106],
      [-1.0761406668405233, -0.9812572320940955, 1.0510634288286071, -1.4203576079753408, 0.2339650344716178, 0.9772807318199377, 0.6811464487329869],
    ]

    # Set up action server
    if with_as:
      action_name = "pointcloud_scan_action"
      self._as = actionlib.SimpleActionServer(action_name, ScanSceneAction, execute_cb=self.execute_cb, auto_start=False)

    # Create publisher for stitched point cloud
    self.stitched_point_cloud_pub = rospy.Publisher('/cloud_stitched', PointCloud2, queue_size=10)

    # Create publisher for selected grasp
    self.selected_grasp_pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=10)

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
    box_pose = PoseStamped()
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
    result = ScanSceneResult()

    if goal.num_scan_poses > len(self.scan_joints):
      rospy.info("Invalid goal set")
      self._as.set_aborted(result)
      return

    captured_clouds = []
    for i in range(goal.num_scan_poses):
   
      if self._as.is_preempt_requested():
        rospy.loginfo("Got preempted")
        self._as.set_preempted()
        return
  
      # self.go_to_pose_goal(self.scan_poses[i])
      self.pointcloud_data = None
      self.go_to_joint_goal(self.scan_joints[i], wait_for_feedback=False)
      rospy.sleep(0.5)

      cloud = self.capture_point_cloud()
      captured_clouds.append(cloud)

    stitched_cloud = self.stitch_point_clouds(captured_clouds)
    self.stitched_point_cloud_pub.publish(stitched_cloud)
    rospy.loginfo("Stitched point clouds") 

    try:
      grasp_candidates = rospy.wait_for_message("/detect_grasps/clustered_grasps", GraspConfigList, timeout=30)
    except rospy.ROSException:
      self._as.set_aborted(result)
      return

    if len(grasp_candidates.grasps) == 0:
      # No grasps detected
      self._as.set_aborted(result)
      return

    grasp_pose = self.select_grasp_pose(grasp_candidates)
    self.selected_grasp_pub.publish(grasp_pose)

    rospy.loginfo("Scanning action succeeded")
    result.selected_grasp_pose = grasp_pose
    self._as.set_succeeded(result)

  def point_cloud_cb(self, data):
    # rospy.loginfo("Received point cloud with timestamp %s", data.header.stamp)
    self.pointcloud_data = copy.deepcopy(data)
     
  def select_grasp_pose(self, grasp_config_list):
    grasp = grasp_config_list.grasps[0]

    x_axis = np.r_[grasp.axis.x, grasp.axis.y, grasp.axis.z]
    y_axis = -np.r_[grasp.binormal.x, grasp.binormal.y, grasp.binormal.z]  # TODO(mbreyer) HACK fix this
    z_axis = np.r_[grasp.approach.x, grasp.approach.y, grasp.approach.z]
    rot = Rotation.from_dcm(np.vstack([x_axis, y_axis, z_axis]).T)
    if np.linalg.det(rot.as_dcm()) < 0:
      rospy.loginfo("Grasp pose vectors not a right-handed system. Flipping y-axis.")
      y_axis = -y_axis
      rot = Rotation.from_dcm(np.vstack([x_axis, y_axis, z_axis]).T)
    offset = rot.apply([0, 0, 0.04])  # GPD defines points at the hand palm, not the fingertip
  
    # If x component is < 0, rotate around z by 180 deg
    if x_axis[0] < 0:
      rospy.loginfo("Flipped grasp pose. x-axis was pointing in negative direction.")
      flip = Rotation.from_euler('z', 180, degrees=True)
      rot = rot*flip

    quat = rot.as_quat()

    pose = Pose()
    pose.position.x = grasp.position.x + offset[0]
    pose.position.y = grasp.position.y + offset[1]
    pose.position.z = grasp.position.z + offset[2]

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "panda_base"

    return pose_stamped


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

  def capture_point_cloud(self):
    if self.pointcloud_data is None:
      rospy.logwarn("Didn't receive any pointcloud data yet.")
      return None
    point_cloud_msg = self.transform_pointcloud(self.pointcloud_data)
    self.pointcloud_data = None
    rospy.loginfo("Captured point cloud")
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
