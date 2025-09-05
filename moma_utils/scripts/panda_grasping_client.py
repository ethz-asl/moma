#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Empty
from std_srvs.srv import Trigger, TriggerResponse

# https://github.com/mbreyer/robot_helpers/blob/main/robot_helpers/ros/moveit.py

from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient
from moma_utils.ros.conversions import to_pose_msg
from moma_utils.ros.conversions import normalize_quaternion
from moma_utils.srv import PoseTarget, PoseTargetResponse, GraspTarget, GraspTargetResponse


class PandaGraspController(object):
    def __init__(self):
        rospy.init_node("panda_grasping_interface")

        self.table_top_link = rospy.get_param("~table_top_frame")
        self.ee_frame = rospy.get_param("~ee_frame")
        # if depth and color are aligned
        # self.camera_frame = "wrist_camera_color_optical_frame"
        # if depth and color are NOT aligned
        self.camera_frame = rospy.get_param("~camera_frame")

        # init robot connection
        self.gripper = PandaGripperClient()
        self.moveit_client = MoveItClient("panda_arm")
        self.moveit_client.move_group.set_end_effector_link(self.ee_frame)

        # Add a box to the planning scene to avoid collisions with the table.
        # msg = geometry_msgs.msg.PoseStamped()
        # msg.header.frame_id = self.base_frame
        # msg.pose.position.x = 0.4
        # msg.pose.position.z = 0.08
        # self.moveit_ee.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

        self.move_ee_srv = rospy.Service("/panda_grasping_interface/move_ee", PoseTarget, self._move_ee_srv_cb)
        self.grasp_srv = rospy.Service("/panda_grasping_interface/grasp", GraspTarget, self._grasp_srv_cb)
        self.move_camera_srv = rospy.Service("/panda_grasping_interface/move_camera", PoseTarget, self._move_camera_srv_cb)

        self.srv_ready = rospy.Service("/panda_grasping_interface/move_to_ready", Trigger, self._srv_move_to_ready)
        self.srv_grasp = rospy.Service("/panda_grasping_interface/gripper_grasp", Trigger, self._srv_gripper_grasp)
        self.srv_open  = rospy.Service("/panda_grasping_interface/gripper_open",  Trigger, self._srv_gripper_open)

        self.reset_map = rospy.ServiceProxy("reset_map", Empty)
        rospy.sleep(1.0)

    def _srv_move_to_ready(self, _req):
        self.moveit_client.move_group.set_end_effector_link(self.ee_frame)
        success = self.moveit_client.goto("ready")
        return TriggerResponse(success=bool(success), message="")

    def _srv_gripper_grasp(self, _req):
        # Using a fixed width/force as in your original method.
        # Adjust if you want this to be parameterized via a custom srv.
        self.gripper.grasp(width=0.0, force=20.0)
        success = self.gripper.read() > 0.004
        return TriggerResponse(success=bool(success), message="")

    def _srv_gripper_open(self, _req):
        self.gripper.move(0.08)
        return TriggerResponse(success=bool(True), message="")

    def _grasp_util(self, req: GraspTarget):
        # open the gripper
        self.gripper.move(0.08)

        # go to pregrasp pose
        pose_stamped = req.pregrasp_pose
        # mirror your old callback’s behavior
        self.moveit_client.move_group.set_end_effector_link(self.ee_frame)
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        result: bool = self.moveit_client.goto(pose_stamped)
        if not result: return False, "pregrasp_movement_fail"

        # go to grasp
        pose_stamped = req.grasp_pose
        # mirror your old callback’s behavior
        self.moveit_client.move_group.set_end_effector_link(self.ee_frame)
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        result: bool = self.moveit_client.gotoL(pose_stamped)
        if not result: return False, "grasp_movement_fail"

        # execute grasp
        self.gripper.grasp(width=0.0, force=20.0)
        # result = self.gripper.read() > 0.004

        # go away
        pose_stamped = req.postgrasp_pose
        # mirror your old callback’s behavior
        self.moveit_client.move_group.set_end_effector_link(self.ee_frame)
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        result: bool = self.moveit_client.gotoL(pose_stamped)
        if not result: return False, "postgrasp_movement_fail"

        # try to close the gripper again
        self.gripper.grasp(width=0.0, force=10.0)
        # this is grasp success (i.e. try to close the hand. If there is an object, it will not close)
        result = self.gripper.read() > 0.004
        if not result: return False, "grasp_empty"
        return True, "grasp_successful"

    def _grasp_srv_cb(self, req: GraspTarget) -> GraspTargetResponse:
        result, msg = self._grasp_util(req)
        return PoseTargetResponse(result, msg)

    def _move_ee_srv_cb(self, req: PoseTarget) -> PoseTargetResponse:
        pose_stamped = req.pose
        # mirror your old callback’s behavior
        self.moveit_client.move_group.set_end_effector_link(self.ee_frame)
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)

        result: bool = self.moveit_client.goto(pose_stamped)
        return PoseTargetResponse(result, "success")

    def _move_camera_srv_cb(self, req: PoseTarget) -> PoseTargetResponse:
        pose_stamped = req.pose
        # mirror your old callback’s behavior
        self.moveit_client.move_group.set_end_effector_link(self.camera_frame)
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)

        result: bool = self.moveit_client.goto(pose_stamped)
        return PoseTargetResponse(result, "success")


if __name__ == "__main__":
    node = PandaGraspController()
    rospy.spin()
