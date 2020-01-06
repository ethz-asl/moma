#!/usr/bin/env python

import actionlib
from fetch_demo.msg import ApproachAction, ApproachResult
import rospy


class ApproachActionServer:
    """
        When called, this action should find a collision free position for the robot,
        as close as possible to the target location (target object), facing it, and navigate
        the robot there using the navigation action.
    """

    def __init__(self):
        action_name = "approach_action"
        self.action_server = actionlib.SimpleActionServer(
            action_name, ApproachAction, execute_cb=self.approach_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Approach action server started.")

    def approach_cb(self, msg):
        rospy.loginfo("Start approaching object")
        result = ApproachResult()

        # Find position for robot, close to the target location

        # Move there using the navigation action

        rospy.sleep(2.0)

        rospy.loginfo("Finished approach")
        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("approach_action_node")
    action = ApproachActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
