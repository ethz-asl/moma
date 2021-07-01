#!/usr/bin/env python

from __future__ import print_function

from actionlib import SimpleActionServer
import numpy as np
import rospy

from grasp_demo.msg import DropAction, DropResult
from moma_utils.spatial import Rotation, Transform
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaGripperClient


class DropActionNode(object):
    """Drops the object back into the workspace with a random offset.
    """

    def __init__(self):
        self.load_parameters()
        self.moveit = MoveItClient("panda_arm")
        self.gripper = PandaGripperClient()
        self.action_server = SimpleActionServer(
            "drop_action", DropAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.action_server.start()

        rospy.loginfo("Drop action server ready")

    def load_parameters(self):
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_drop")

    def execute_cb(self, goal):
        rospy.loginfo("Dropping object")

        ori = Rotation.from_quat([1.000, 0.0, 0.0, 0.0])
        pos = [0.307, 0.0, 0.487]
        pos[0] += np.random.uniform(0.05, 0.25)
        pos[1] += np.random.uniform(-0.2, 0.0)
        pos[2] -= 0.3

        self.moveit.goto(Transform(ori, pos), velocity_scaling=self.velocity_scaling)
        self.gripper.release()
        self.action_server.set_succeeded(DropResult())


def main():
    rospy.init_node("drop_action_node")
    DropActionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
