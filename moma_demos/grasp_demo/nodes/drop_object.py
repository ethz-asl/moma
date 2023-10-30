#!/usr/bin/env python3

import argparse
from actionlib import SimpleActionServer
import rospy

from grasp_demo.msg import DropAction, DropResult
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaGripperClient


class DropActionNode(object):
    """Drops the object back into the workspace with a random offset."""

    def __init__(self, arm_id):
        self.load_parameters()
        self.moveit = MoveItClient(f"{arm_id}_arm")
        self.gripper = PandaGripperClient()
        self.action_server = SimpleActionServer(
            "drop_action", DropAction, execute_cb=self.drop_object, auto_start=False
        )
        self.action_server.start()

        rospy.loginfo("Drop action server ready")

    def load_parameters(self):
        self.velocity_scaling = rospy.get_param("moma_demo/arm_velocity_scaling_drop")

    def drop_object(self, goal):
        rospy.loginfo("Dropping object")
        i = rospy.get_param("moma_demo/workspace", 0)
        drop_joints = rospy.get_param("moma_demo/workspaces")[i]["drop_joints"]
        self.moveit.goto(drop_joints, velocity_scaling=self.velocity_scaling)
        self.gripper.release()
        self.action_server.set_succeeded(DropResult())


def main():
    rospy.init_node("drop_action_node")
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm_id", type=str, default="panda")
    args = parser.parse_known_args()
    DropActionNode(args.arm_id)
    rospy.spin()


if __name__ == "__main__":
    main()
