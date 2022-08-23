#!/usr/bin/env python

from geometry_msgs.msg import Pose
from mobile_manip_demo.robot_interface import Recharge, RobotAtPose

import rospy
import tf2_ros

import numpy as np


class RechargeNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        self.target_pose = Pose()
        self.target_pose.position.x = 0.5
        self.target_pose.position.y = -2.0
        self.target_pose.position.z = 0.0
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = 0.0
        self.target_pose.orientation.w = 1.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def recharge(self):
        self.recharge_action.cancel_goal()
        self.recharge_action.initialize_recharge(self.target_pose)

        while not rospy.is_shutdown():
            status = self.recharge_action.get_recharge_status()
            rospy.loginfo("recharge RUNNING")
            if status == 3:
                rospy.loginfo("recharge SUCCESS")
                rospy.signal_shutdown("Success, shutting down!")
            else:
                rospy.loginfo("recharge FAILURE")
                rospy.signal_shutdown("Failure, shutting down!")


def main():
    rospy.init_node("recharge_tester_node")
    node = RechargeNode()

    try:
        node.recharge()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
