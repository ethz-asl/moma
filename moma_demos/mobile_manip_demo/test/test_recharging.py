#!/usr/bin/env python

from geometry_msgs.msg import Pose
from mobile_manip_demo.robot_interface import BatteryLv, Recharge, RobotAtPose

import rospy
import tf2_ros

import numpy as np


class RechargeNode:
    def __init__(self):
        """Initialize ROS nodes."""
        self.recharge_action = Recharge()
        self.recharge_condition = BatteryLv()

    def recharge(self):
        self.recharge_action.cancel_goal()
        self.recharge_action.initialize_recharge()

        while not rospy.is_shutdown():
            rospy.Rate(1).sleep()
            status = self.recharge_action.get_recharge_status()

            rospy.loginfo(
                f'Battery below 30%: {self.recharge_condition.battery_lv("lower", 30)}'
            )

            if status == 0 or status == 1:
                rospy.loginfo("recharge RUNNING")
            elif status == 3:
                rospy.loginfo("recharge SUCCESS")
                rospy.signal_shutdown("Success")
            else:
                rospy.loginfo("recharge FAILURE")
                rospy.signal_shutdown("Failure")


def main():
    rospy.init_node("recharge_tester_node")
    node = RechargeNode()

    try:
        node.recharge()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
