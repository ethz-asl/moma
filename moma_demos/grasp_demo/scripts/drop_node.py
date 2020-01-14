#!/usr/bin/env python

from __future__ import print_function

import sys

from actionlib import SimpleActionServer
import rospy

from grasp_demo.msg import DropAction, DropResult
from grasp_demo.utils import create_robot_connection


class DropActionNode(object):
    def __init__(self):
        self.robot_commander = create_robot_connection(sys.argv[1])

        self._as = SimpleActionServer(
            "drop_action", DropAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()

        rospy.loginfo("Drop action server ready")

    def execute_cb(self, goal):
        rospy.loginfo("Dropping action was triggered")

        drop_pose = rospy.get_param("grasp_demo")["drop_pose"]
        self.robot_commander.goto_pose_target(drop_pose, max_velocity_scaling=0.4)

        rospy.sleep(0.5)  # wait for the operator's hand to be placed under the EE

        # self.robot_commander.move_gripper(width=0.07)
        self.robot_commander.release()

        rospy.sleep(0.2)

        home_joints = self.robot_commander.move_group.get_named_target_values("home")
        self.robot_commander.goto_joint_target(home_joints, max_velocity_scaling=0.4)

        rospy.loginfo("Dropping action succeeded")
        self._as.set_succeeded(DropResult())


def main():
    try:
        rospy.init_node("drop_action_node")
        DropActionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
