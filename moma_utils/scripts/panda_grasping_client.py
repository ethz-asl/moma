#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros

from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Empty
from std_srvs.srv import Trigger, TriggerResponse

# https://github.com/mbreyer/robot_helpers/blob/main/robot_helpers/ros/moveit.py

from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient
from moma_utils.ros.conversions import (
    from_pose_msg,
    from_transform_msg,
    normalize_quaternion,
    to_pose_msg,
)
from moma_utils.srv import (
    PoseTarget,
    PoseTargetResponse,
    GraspTarget,
    GraspTargetResponse,
    JointTarget,
    JointTargetResponse,
    JointValues,
    JointValuesResponse,
)


class PandaGraspController(object):
    def __init__(self):
        rospy.init_node("panda_grasping_interface")

        self.table_top_link = "table_top"
        self.command_frame = "panda_default_ee"
        self.ee_frame = "panda_default_ee"
        # if depth and color are NOT aligned
        # self.camera_frame = "wrist_camera_depth_optical_frame"
        # if depth and color are aligned (our use case)
        self.camera_frame = "wrist_camera_color_optical_frame"

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._T_ee_from_command = None
        self._T_camera_from_command = None

        # init robot connection
        self.gripper = PandaGripperClient()
        self.moveit_client = MoveItClient("panda_arm")
        self.moveit_client.move_group.set_end_effector_link(self.command_frame)
        self.moveit_client.move_group.set_pose_reference_frame(self.table_top_link)

        # Add a box to the planning scene to avoid collisions with the table.
        # msg = geometry_msgs.msg.PoseStamped()
        # msg.header.frame_id = self.base_frame
        # msg.pose.position.x = 0.4
        # msg.pose.position.z = 0.08
        # self.moveit_ee.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

        self.move_ee_srv = rospy.Service("/panda_grasping_interface/move_ee", PoseTarget, self._move_ee_srv_cb)
        self.grasp_srv = rospy.Service("/panda_grasping_interface/grasp", GraspTarget, self._grasp_srv_cb)
        self.move_camera_srv = rospy.Service("/panda_grasping_interface/move_camera", PoseTarget, self._move_camera_srv_cb)
        self.move_joints_srv = rospy.Service("/panda_grasping_interface/move_joints", JointTarget, self._move_joints_srv_cb)
        self.get_joint_values_srv = rospy.Service("/panda_grasping_interface/get_joint_values", JointValues, self._get_joint_values_srv_cb)

        self.srv_ready = rospy.Service("/panda_grasping_interface/move_to_ready", Trigger, self._srv_move_to_ready)
        self.srv_grasp = rospy.Service("/panda_grasping_interface/gripper_grasp", Trigger, self._srv_gripper_grasp)
        self.srv_open  = rospy.Service("/panda_grasping_interface/gripper_open",  Trigger, self._srv_gripper_open)

        self.reset_map = rospy.ServiceProxy("reset_map", Empty)
        rospy.sleep(1.0)
        self._initialize_command_transforms()

    def _srv_move_to_ready(self, _req):
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
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        T_world_ee = from_pose_msg(pose_stamped.pose)
        T_world_command = T_world_ee * self._T_ee_from_command
        pose_stamped.pose = to_pose_msg(T_world_command)
        result: bool = self.moveit_client.goto(pose_stamped)
        if not result: return False, "pregrasp_movement_fail"

        # go to grasp
        pose_stamped = req.grasp_pose
        # mirror your old callback’s behavior
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        T_world_ee = from_pose_msg(pose_stamped.pose)
        T_world_command = T_world_ee * self._T_ee_from_command
        pose_stamped.pose = to_pose_msg(T_world_command)
        result: bool = self.moveit_client.goto(pose_stamped)
        if not result: return False, "grasp_movement_fail"

        # execute grasp
        self.gripper.grasp(width=0.0, force=10.0)
        # result = self.gripper.read() > 0.004

        # go away
        pose_stamped = req.postgrasp_pose
        # mirror your old callback’s behavior
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        T_world_ee = from_pose_msg(pose_stamped.pose)
        T_world_command = T_world_ee * self._T_ee_from_command
        pose_stamped.pose = to_pose_msg(T_world_command)
        result: bool = self.moveit_client.goto(pose_stamped)
        if not result: return False, "postgrasp_movement_fail"

        # try to close the gripper again
        self.gripper.grasp(width=0.0, force=10.0)
        # this is grasp success (i.e. try to close the hand. If there is an object, it will not close)
        result = self.gripper.read() > 0.004
        if not result: return False, "grasp_empty"
        return True, "grasp_successful"

    def _grasp_srv_cb(self, req: GraspTarget) -> GraspTargetResponse:
        result, msg = self._grasp_util(req)
        return GraspTargetResponse(result, msg)

    def _move_ee_srv_cb(self, req: PoseTarget) -> PoseTargetResponse:
        if self._T_ee_from_command is None:
            return PoseTargetResponse(False, "command frame transform unavailable")

        pose_stamped = req.pose
        # mirror your old callback’s behavior
        
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)
        T_world_ee = from_pose_msg(pose_stamped.pose)
        T_world_command = T_world_ee * self._T_ee_from_command
        pose_stamped.pose = to_pose_msg(T_world_command)

        result: bool = self.moveit_client.goto(pose_stamped)
        return PoseTargetResponse(result, "success")

    def _move_joints_srv_cb(self, req: JointTarget) -> JointTargetResponse:
        joint_target = list(req.joint_positions)
        if not joint_target:
            return JointTargetResponse(False, "joint target empty")

        success: bool = self.moveit_client.goto(joint_target)
        message = "success" if success else "plan_failed"
        return JointTargetResponse(bool(success), message)

    def _move_camera_srv_cb(self, req: PoseTarget) -> PoseTargetResponse:
        if self._T_camera_from_command is None:
            return PoseTargetResponse(False, "camera frame transform unavailable")

        pose_stamped = req.pose
        # mirror your old callback’s behavior
        pose_stamped.header.frame_id = self.table_top_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped = normalize_quaternion(pose_stamped)

        T_world_camera = from_pose_msg(pose_stamped.pose)
        T_world_command = T_world_camera * self._T_camera_from_command
        pose_stamped.pose = to_pose_msg(T_world_command)

        result: bool = self.moveit_client.goto(pose_stamped)
        return PoseTargetResponse(result, "success")

    def _get_joint_values_srv_cb(self, _req: JointValues) -> JointValuesResponse:
        joint_positions = self.moveit_client.move_group.get_current_joint_values()
        return JointValuesResponse(joint_positions=joint_positions, success=True, message="success")

    def _initialize_command_transforms(self):
        timeout = rospy.Duration(5.0)
        self._T_ee_from_command = self._lookup_transform(
            self.ee_frame, self.command_frame, timeout
        )
        self._T_camera_from_command = self._lookup_transform(
            self.camera_frame, self.command_frame, timeout
        )

    def _lookup_transform(self, target_frame, source_frame, timeout):
        try:
            transform_msg = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), timeout
            )
        except tf2_ros.TransformException as exc:
            rospy.logfatal(
                "Failed to lookup transform from %s to %s: %s",
                source_frame,
                target_frame,
                exc,
            )
            raise
        return from_transform_msg(transform_msg.transform)


if __name__ == "__main__":
    node = PandaGraspController()
    rospy.spin()
