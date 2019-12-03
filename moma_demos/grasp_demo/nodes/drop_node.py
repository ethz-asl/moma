#!/usr/bin/env python

from __future__ import print_function

import actionlib
import rospy

from grasp_demo.msg import DropAction, DropResult
from grasp_demo.panda_commander import PandaCommander


class DropActionNode(object):

    def __init__(self):
        # Panda commander
        self.panda_commander = PandaCommander("panda_arm")

        # Poses [x y z qx qy qz qw]
        self.drop_pose = [0.267, -0.379, 0.454, 0.937, -0.349, 0.004, -0.006]

        # Set up action server
        action_name = "drop_action"
        self._as = actionlib.SimpleActionServer(action_name, DropAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        rospy.loginfo("Drop action server ready")

    def execute_cb(self, goal):
        rospy.loginfo("Dropping action was triggered")
   
        self.panda_commander.goto_pose_target(self.drop_pose, max_velocity_scaling=0.5)

        rospy.sleep(1.0)  # wait for the operator's hand to be placed under the EE

        self.panda_commander.move_gripper(width=0.07)

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
