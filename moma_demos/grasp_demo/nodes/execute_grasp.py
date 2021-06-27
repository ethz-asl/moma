#!/usr/bin/env python

from actionlib import SimpleActionServer
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy

from grasp_demo.msg import GraspAction, GraspResult
from moma_utils.ros.conversions import from_pose_msg, to_pose_stamped_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient
from moma_utils.spatial import Transform


class GraspExecutionAction(object):
    """Execute a grasp specified by the action goal using MoveIt.
    """

    def __init__(self):
        self._load_parameters()
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()
        self.moveit = MoveItClient("panda_arm")

        self.pregrasp_pub = rospy.Publisher("pregrasp_pose", PoseStamped, queue_size=10)

        self.action_server = SimpleActionServer(
            "grasp_execution_action",
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        rospy.loginfo("Grasp action server ready")

    def _load_parameters(self):
        self.base_frame = rospy.get_param("moma_demo/base_frame_id")
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_grasp")
        self.pregrasp_offset_z = rospy.get_param("moma_demo/pregrasp_offset_z")
        self.grasp_offset_z = rospy.get_param("moma_demo/grasp_offset_z")

    def execute_cb(self, goal):
        rospy.loginfo("Received grasp pose")
        T_base_grasp = from_pose_msg(goal.target_grasp_pose.pose)
        T_grasp_offset = Transform.translation([0.0, 0.0, self.grasp_offset_z])
        T_base_grasp = T_base_grasp * T_grasp_offset
        T_grasp_pregrasp = Transform.translation([0.0, 0.0, self.pregrasp_offset_z])
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp

        # Visualize pre-grasp pose
        self.pregrasp_pub.publish(to_pose_stamped_msg(T_base_pregrasp, self.base_frame))

        self.gripper.release()
        self.moveit.goto("ready", self.velocity_scaling)
        self.moveit.goto(T_base_pregrasp, self.velocity_scaling)
        self.moveit.goto(T_base_grasp, self.velocity_scaling)

        if self.arm.has_error:
            self.action_server.set_aborted()
            return

        self.gripper.grasp()

        if self.arm.has_error:
            self.action_server.set_aborted()
            return

        self.moveit.goto(T_base_pregrasp, self.velocity_scaling)

        if self.gripper.read() > 0.01:
            self.action_server.set_succeeded(GraspResult())
        else:
            self.action_server.set_aborted()


if __name__ == "__main__":
    rospy.init_node("grasp_execution_node")
    GraspExecutionAction()
    rospy.spin()
