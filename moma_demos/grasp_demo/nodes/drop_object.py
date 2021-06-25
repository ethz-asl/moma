#!/usr/bin/env python

from __future__ import print_function

import numpy as np

from actionlib import SimpleActionServer
import rospy

from panda_control.panda_commander import PandaCommander
from grasp_demo.msg import DropAction, DropResult


class DropActionNode(object):
    def __init__(self):
        self.arm = PandaCommander()
        self.load_parameters()
        self.action_server = SimpleActionServer(
            "drop_action", DropAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.action_server.start()

        rospy.loginfo("Drop action server ready")

    def load_parameters(self):
        self.home_joints = rospy.get_param("moma_demo/home_joints_arm")
        self.drop_joints = rospy.get_param("moma_demo/drop_joints_arm")
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_drop")

    def execute_cb(self, goal):
        rospy.loginfo("Dropping action was triggered")

        # Drop at a random location within the workspace
        drop_pose = [0.307, 0.0, 0.487, 1.000, 0.0, 0.0, 0.0]
        drop_pose[0] += np.random.uniform(0.05, 0.25)
        drop_pose[1] += np.random.uniform(-0.1, 0.1)
        drop_pose[2] -= 0.2

        rospy.loginfo("Moving to drop pose")
        self.arm.goto_pose_target(drop_pose, max_velocity_scaling=self.velocity_scaling)
        rospy.sleep(2.0)

        self.arm.release()

        rospy.loginfo("Dropping action succeeded")
        self.action_server.set_succeeded(DropResult())


def main():
    rospy.init_node("drop_action_node")
    DropActionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
