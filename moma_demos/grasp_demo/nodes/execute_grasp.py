#!/usr/bin/env python3

import argparse
from actionlib import SimpleActionServer
from geometry_msgs.msg import PoseStamped
import rospy

from grasp_demo.msg import GraspAction
from moma_utils.ros.conversions import from_pose_msg, to_pose_stamped_msg
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient
from moma_utils.spatial import Transform


class GraspExecutionAction(object):
    """Execute a grasp specified by the action goal using MoveIt."""

    def __init__(self, arm_id):
        self.load_parameters()
        # self.moveit = MoveItClient(f"{arm_id}_arm")
        self.moveit = MoveItClient(f"{arm_id}_manipulator")
        self.arm = PandaArmClient()
        self.gripper = PandaGripperClient()

        self.action_server = SimpleActionServer(
            "grasp_execution_action",
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.start()

        self.moveit_target_pub = rospy.Publisher("target", PoseStamped, queue_size=10)

        rospy.loginfo("Grasp action server ready")

    def load_parameters(self):
        self.base_frame = rospy.get_param("moma_demo/base_frame_id")
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_grasp")
        self.ee_grasp_offset_z = rospy.get_param("moma_demo/ee_grasp_offset_z")

    def execute_cb(self, goal):
        rospy.loginfo("Received grasp pose")
        T_base_grasp = from_pose_msg(goal.target_grasp_pose.pose)

        T_grasp_ee_offset = Transform.translation([0.0, 0.0, -self.ee_grasp_offset_z])

        rospy.loginfo("Executing grasp")
        self.gripper.release()
        self.moveit.goto("ready", self.velocity_scaling)

        rospy.loginfo("Moving to pregrasp pose")
        target = T_base_grasp * Transform.translation([0, 0, -0.04]) * T_grasp_ee_offset
        self.moveit_target_pub.publish(to_pose_stamped_msg(target, self.base_frame))
        success = self.moveit.goto(target, self.velocity_scaling)

        rospy.loginfo("Moving to grasp pose")
        target = T_base_grasp * T_grasp_ee_offset
        self.moveit_target_pub.publish(to_pose_stamped_msg(target, self.base_frame))
        self.moveit.gotoL(target, self.velocity_scaling)

        if self.arm.has_error:
            rospy.loginfo("Robot error. Aborting.")
            self.action_server.set_aborted()
            return

        rospy.loginfo("Attempting grasp")
        self.gripper.grasp()

        if self.arm.has_error:
            rospy.loginfo("Robot error. Aborting.")
            self.action_server.set_aborted()
            return

        rospy.loginfo("Lifting object")
        target = Transform.translation([0, 0, 0.2]) * T_base_grasp * T_grasp_ee_offset
        self.moveit_target_pub.publish(to_pose_stamped_msg(target, self.base_frame))
        self.moveit.gotoL(target, self.velocity_scaling)

        if self.gripper.read() > 0.002:
            rospy.loginfo("Object grasped successfully")
            self.action_server.set_succeeded()
        else:
            rospy.logwarn("Nothing detected in gripper")
            self.action_server.set_aborted()


if __name__ == "__main__":
    rospy.init_node("grasp_execution_node")
    parser = argparse.ArgumentParser()
    parser.add_argument("-arm_id", type=str, default="panda")
    args, _ = parser.parse_known_args()
    GraspExecutionAction(args.arm_id)
    rospy.spin()
