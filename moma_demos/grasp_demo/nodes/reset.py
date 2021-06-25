#!/usr/bin/env python

from __future__ import print_function

import geometry_msgs.msg
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from panda_control.panda_commander import PandaCommander


class ResetNode(object):
    def __init__(self):
        self.arm = PandaCommander()
        self.home_joints = rospy.get_param("moma_demo/home_joints_arm")
        self.create_planning_scene()
        rospy.Service("reset", Trigger, self.reset)
        rospy.loginfo("Reset service ready")

    def create_planning_scene(self):
        # collision box for table
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x = 0.4
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.06
        self.arm.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

    def reset(self, req):
        rospy.loginfo("Reset")
        if self.arm.has_error:
            self.arm.recover()
        self.arm.goto_joint_target(self.home_joints, max_velocity_scaling=0.2)
        return TriggerResponse()


def main():
    rospy.init_node("reset")
    ResetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
