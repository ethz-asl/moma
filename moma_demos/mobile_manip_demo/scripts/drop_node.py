#!/usr/bin/env python3
"""ROS node exposing the action for the Drop Skill."""

import numpy as np

import rospy

from actionlib import SimpleActionServer

from mobile_manip_demo import environment as env
from mobile_manip_demo.skill_template import Skill
from mobile_manip_demo.msg import DropAction, DropResult
from mobile_manip_demo.srv import ForceDrop, ForceDropRequest

import moma_utils.ros.conversions as conv
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaGripperClient


class DropSkill(Skill):
    """Drops the target object at a given position."""

    def __init__(self):
        super().__init__()
        # Parameters
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_drop")
        self.drop_joints = rospy.get_param("moma_demo/drop_joints")
        self.base_frame_id = rospy.get_param("moma_demo/base_frame_id")
        self.object_type = rospy.get_param("moma_demo/object_type")

        self.moveit = MoveItClient("panda_arm")
        self.gripper = PandaGripperClient()

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

    def drop_object(self, goal):

        if goal.target_object_pose is None:
            target_pose = self.drop_joints
        else:
            # target pose is expected to be in the panda base frame
            # but the goal is most likely in map frame
            goal_as_transform = conv.from_pose_msg(goal.target_object_pose.pose)
            goal_frame = goal.target_object_pose.header.frame_id
            msg = self.compute_tf(goal_frame, self.base_frame_id)
            if msg is None:
                self.report_failure(DropResult(), "Transformation Error")
                raise ValueError("Could not get the transformation")

            tf_to_base = conv.from_transform_msg(msg.transform)
            target_pose = tf_to_base.__mul__(goal_as_transform)

        rospy.logwarn(
            f"Dropping object at pose {target_pose.to_list()} in frame {goal_frame}"
        )
        self.moveit.goto(target_pose, velocity_scaling=self.velocity_scaling)
        # Check that the EE is at the target pose
        # TODO: this should be replaced by the MoveIt action result server
        msg = self.compute_tf("panda_EE", self.base_frame_id)
        msg_as_tf = conv.from_transform_msg(msg.transform)
        if (
            np.linalg.norm(
                np.array(target_pose.to_list()[:3]) - np.array(msg_as_tf.to_list()[:3])
            )
            > 0.2
        ):
            self.report_failure(DropResult(), "Could not reach drop goal pose")
            return

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
        moveit_client.goto("post_place")

        if response.success:
            self.report_success(DropResult(), "Finished successfully")
        else:
            self.report_failure(DropResult(), "Drop execution failed")


if __name__ == "__main__":
    rospy.init_node("drop_skill")
    ds = DropSkill()
    rospy.spin()
