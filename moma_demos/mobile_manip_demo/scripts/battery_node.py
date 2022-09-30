#!/usr/bin/env python3
"""ROS node exposing the action for the Recharge Skill."""

from copy import copy

import rospy

from actionlib import SimpleActionServer
from std_msgs.msg import Int32

from mobile_manip_demo.skill_template import Skill
from mobile_manip_demo.msg import RechargeAction, RechargeResult


class RechargeSkill(Skill):
    """Handle the battery management."""

    def __init__(self):
        super().__init__()
        # Parameters
        self.battery_lv = rospy.get_param("moma_demo/battery_lv")
        self.drop_rate = rospy.get_param("moma_demo/battery_drop_rate")
        self.charge_rate = rospy.get_param("moma_demo/battery_charge_rate")
        self.current_lv = copy(self.battery_lv)

        task_type = rospy.get_param("moma_demo/experiment")
        if task_type == 1:
            self.drop_rate = 0

        # Action server
        self.action_server = SimpleActionServer(
            "/recharge",
            RechargeAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        rospy.loginfo("Initializing recharging action server")
        self.action_server.start()
        self.recharging = False

        self.battery_pub = rospy.Publisher("/battery_level", Int32, queue_size=10)

    def publish_battery_lv(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.recharging:
                # self.current_lv += self.charge_rate
                # Instantaneous recharging:
                self.current_lv = copy(self.battery_lv)
                self.recharging = False
            else:
                self.current_lv -= self.drop_rate
            self.battery_pub.publish(self.current_lv)
            rate.sleep()

    def execute_callback(self, goal):
        rospy.logwarn("Recharging the batteries!")
        self.recharging = True
        # while self.current_lv < self.battery_lv:
        #     rospy.sleep(2)
        # self.recharging = False

        rospy.logwarn("Recharged!")
        self.report_success(RechargeResult(), "Battery recharged!")


if __name__ == "__main__":
    rospy.init_node("recharge_skill")
    rs = RechargeSkill()
    rs.publish_battery_lv()
    rospy.spin()
