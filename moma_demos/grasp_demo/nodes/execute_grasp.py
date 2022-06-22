#!/usr/bin/env python3

from actionlib import SimpleActionServer
import rospy

from grasp_demo.msg import GraspAction, GraspResult
from moma_utils.ros.conversions import from_pose_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient
from moma_utils.spatial import Transform


class GraspExecutionAction(object):
    """Execute a grasp specified by the action goal using MoveIt."""

    def __init__(self):
        self.load_parameters()
        self.moveit = MoveItClient("panda_arm")
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()

        self.action_server = SimpleActionServer(
            "grasp_execution_action",
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        rospy.loginfo("Grasp action server ready")

    def load_parameters(self):
        self.base_frame = rospy.get_param("moma_demo/base_frame_id")
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_grasp")
        self.ee_grasp_offset_z = rospy.get_param("moma_demo/ee_grasp_offset_z")

    def execute_cb(self, goal):
        rospy.loginfo("Received grasp pose")
        T_base_grasp = from_pose_msg(goal.target_grasp_pose.pose)
        T_grasp_ee_offset = Transform.translation([0.0, 0.0, -self.ee_grasp_offset_z])
        T_base_grasp = T_base_grasp * T_grasp_ee_offset
        T_grasp_pregrasp = Transform.translation([0.0, 0.0, -0.03])
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp

        self.gripper.release()
        self.moveit.goto("ready", self.velocity_scaling)
        if self.arm.has_error:
            self.arm.recover()
        self.moveit.goto(T_base_pregrasp, self.velocity_scaling)
        self.moveit.goto(T_base_grasp, self.velocity_scaling)

        if self.arm.has_error:
            self.arm.recover()
            return

        self.gripper.grasp()

        if self.arm.has_error:
            self.arm.recover()
            return

        self.moveit.goto(T_base_pregrasp, self.velocity_scaling)

        if self.gripper.read() > 0.01:
            rospy.loginfo("Object grasped successfully")
        else:
            rospy.logwarn("Nothing detected in gripper")
        self.action_server.set_succeeded(GraspResult())


if __name__ == "__main__":
    rospy.init_node("grasp_execution_node")
    GraspExecutionAction()
    rospy.spin()
