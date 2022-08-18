#!/usr/bin/env python

"""Test the to move the arm."""

from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, PoseStamped
from moma_utils.ros.moveit import MoveItClient
import moma_utils.ros.conversions as conv
from moma_utils.spatial import Transform

import rospy
import tf2_ros

import numpy as np


class ArmControl:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.moveit = MoveItClient("panda_arm")

    def send_move_request(self):
        # Put arm on a named configuration
        self.moveit.goto("ready")
        rospy.sleep(5.0)

        self.moveit.goto([0, 0, 0, -1.57079632679, 0, 1.57079632679, 0.785398163397])
        rospy.sleep(5.0)

        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.position.x = 0.4 + 1e-6
        pose.pose.position.y = 0.0 + 1e-6
        pose.pose.position.z = 0.3 + 1e-6
        pose.pose.orientation.x = 1.0 + 1e-6
        pose.pose.orientation.y = 1e-6
        pose.pose.orientation.z = 1e-6
        pose.pose.orientation.w = 1e-6
        self.moveit.move_group.set_pose_target(pose)
        plan = self.moveit.move_group.plan()
        if type(plan) is tuple:
            plan = plan[1]
        self.moveit.move_group.set_goal_tolerance(0.01)
        success = self.moveit.move_group.execute(plan, wait=True)
        self.moveit.move_group.stop()
        self.moveit.move_group.clear_pose_targets()
        rospy.sleep(5.0)

        translation = [0.4, 0.2, 0.3]
        rotation = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])
        pose = Transform(rotation, translation)
        self.moveit.goto(pose)


def main():
    rospy.init_node("arm_tester_node")
    node = ArmControl()

    try:
        node.send_move_request()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
