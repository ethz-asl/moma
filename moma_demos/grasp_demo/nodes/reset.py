#!/usr/bin/env python

from __future__ import print_function


import rospy
from std_srvs.srv import Trigger, TriggerResponse

from panda_control.panda_commander import PandaCommander


class ResetNode(object):
    def __init__(self):
        self.arm = PandaCommander()
        self.home_joints = rospy.get_param("moma_demo/home_joints_arm")
        rospy.Service("reset", Trigger, self.reset)
        rospy.loginfo("Reset service ready")

    def reset(self, req):
        rospy.loginfo("Reset")
        self.arm.goto_joint_target(self.home_joints, max_velocity_scaling=0.2)
        return TriggerResponse()


def main():
    rospy.init_node("reset")
    ResetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
