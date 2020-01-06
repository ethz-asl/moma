#!/usr/bin/env python

import actionlib
from fetch_demo.msg import DropMoveAction, DropMoveResult
import rospy


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
        self.action_server.start()
        rospy.loginfo("Drop move action server started.")

    def drop_cb(self, msg):
        rospy.loginfo("Start approaching object")
        result = DropMoveResult()

        # Find position for robot, close to the target location

        # Move there using the navigation action

        rospy.sleep(2.0)

        rospy.loginfo("Finished dropping")
        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("drop_move_action_node")
    action = DropActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
