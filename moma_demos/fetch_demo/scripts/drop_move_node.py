#!/usr/bin/env python

import actionlib
from fetch_demo.msg import DropMoveAction, DropMoveResult
import rospy

from grasp_demo.utils import create_robot_connection


class DropActionServer:
    """
        When called, this action should navigate the base and arm to a pre-specified position,
        then open the gripper to drop the grasped object. 
    """

    def __init__(self):
        action_name = "drop_move_action"
        self.action_server = actionlib.SimpleActionServer(
            action_name, DropMoveAction, execute_cb=self.drop_cb, auto_start=False
        )

        self._read_joint_configurations()
        self._connect_yumi()

        self.action_server.start()
        rospy.loginfo("Drop move action server started.")

    def _read_joint_configurations(self):
        # self._search_joints_r = rospy.get_param("search_joints_r")
        self._ready_joints_l = rospy.get_param("ready_joints_l")

    def _connect_yumi(self):
        self._left_arm = create_robot_connection("yumi_left_arm")

    def drop_cb(self, msg):
        rospy.loginfo("Start approaching object")
        result = DropMoveResult()

        # Find position for robot, close to the target location

        # Move there using the navigation action

        self._left_arm.release()

        rospy.loginfo("Finished dropping")
        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("drop_move_action_node")
    DropActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
