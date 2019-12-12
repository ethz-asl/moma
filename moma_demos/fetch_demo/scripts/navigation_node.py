#!/usr/bin/env python

import actionlib
from fetch_demo.msg import NavigationAction, NavigationResult
from geometry_msgs.msg import Pose
import rospy


class NavigationActionServer:
    """
        When called, this action should drive the robot to the desired position/orientation.
    """
    def __init__(self):
        action_name = "navigation_action"
        self.action_server = actionlib.SimpleActionServer(
            action_name, NavigationAction, execute_cb=self.nav_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Navigation action server started.")

    def nav_cb(self, msg):
        # rospy.loginfo("Starting navigation to target location")
        target_pose = msg.target_pose
        result = NavigationResult()

        rospy.sleep(3.0)

        rospy.loginfo("Navigation successful")
        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("navigation_action_node")
    action = NavigationActionServer()
    rospy.spin()

if __name__ == "__main__":
    main()
