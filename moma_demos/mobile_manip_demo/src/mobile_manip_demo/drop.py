#!/usr/bin/env python3

from actionlib import SimpleActionServer
import numpy as np
import rospy
import tf2_ros

from mobile_manip_demo.msg import DropAction, DropResult
from moma_utils.spatial import Rotation, Transform
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaGripperClient
import moma_utils.ros.conversions as conv
from mobile_manip_demo import environment as env
import mobile_manip_demo.msg
from mobile_manip_demo.srv import ForceDrop, ForceDropRequest


class DropSkill:
    """Drops the target object at a given position."""

    def __init__(self):
        self.load_parameters()

        self.moveit = MoveItClient("panda_arm")
        self.gripper = PandaGripperClient()

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        self.action_server = SimpleActionServer(
            "hl_drop",
            DropAction,
            execute_cb=self.drop_object,
            auto_start=False,
        )
        rospy.loginfo("Initializing dropping action server")
        self.action_server.start()

        self.detach_srv = rospy.ServiceProxy("force_drop", ForceDrop)
        self.detach_srv.wait_for_service()

    def load_parameters(self):
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_drop")
        self.drop_joints = rospy.get_param("moma_demo/drop_joints")
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.object_type = rospy.get_param("moma_demo/object_type")

    def drop_object(self, goal):

        if goal.target_object_pose is None:
            target_pose = self.drop_joints
        else:
            # target pose is expected to be in the panda base frame
            # but the goal is most likely in map frame
            goal_as_transform = conv.from_pose_msg(goal.target_object_pose.pose)
            goal_frame = goal.target_object_pose.header.frame_id
            msg = self.__compute_tf(goal_frame, self.base_frame_id)
            if msg is None:
                self.report_failure("Transformation Error")
                raise ValueError("Could not get the transformation")

            tf_to_base = conv.from_transform_msg(msg.transform)
            target_pose = tf_to_base.__mul__(goal_as_transform)

        rospy.logwarn(
            f"Dropping object at pose {target_pose.to_list()} in frame {goal_frame}"
        )
        done = self.moveit.goto(target_pose, velocity_scaling=self.velocity_scaling)
        if not done:
            self.report_failure("Could not reach drop goal pose")
        else:
            self.gripper.release()

            name, link = env.get_item_by_marker(int(goal.goal_id), self.object_type)
            self.drop_request = ForceDropRequest()
            self.drop_request.model_name = name
            self.drop_request.ee_name = "panda"
            self.drop_request.model_link = link
            self.drop_request.ee_link = "panda::panda_leftfinger"
            response = self.detach_srv.call(self.drop_request)

            # go back to ready position
            moveit_client = MoveItClient("panda_arm")
            moveit_client.goto("ready")

            if response.success:
                self.report_success("Finished successfully")
            else:
                self.report_failure("Drop execution failed")

    def report_failure(self, msg):
        result = mobile_manip_demo.msg.DropResult()
        rospy.logerr(msg)
        result.success = False
        result.message = msg
        self.action_server.set_aborted(result)

    def report_preemption(self, msg):
        result = mobile_manip_demo.msg.DropResult()
        rospy.logwarn(msg)
        result.success = False
        result.message = msg
        self.action_server.set_preempted(result)

    def report_success(self, msg: str):
        result = mobile_manip_demo.msg.DropResult()
        rospy.loginfo(msg)
        result.success = True
        result.message = msg
        self.action_server.set_succeeded(result)

    def wait_monitoring_preemption(self, client):
        while not client.wait_for_result(timeout=rospy.Duration(1)):
            if self.action_server.is_preempt_requested():
                client.cancel_goal()
                return False
        return True

    def __compute_tf(self, target_frame: str, reference_frame: str):
        done = False
        attempts = 0
        while not done and attempts < 15:
            try:
                msg = self.tf_buffer.lookup_transform(
                    reference_frame,
                    target_frame,
                    rospy.Time(),
                    rospy.Duration(5),
                )
                done = True
            except Exception:
                attempts += 1
                rospy.logerr("Could not get transform, retrying...")

        if done:
            return msg
        else:
            return None
