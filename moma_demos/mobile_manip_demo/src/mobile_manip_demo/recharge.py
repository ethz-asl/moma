#!/usr/bin/env python3
"""The recharge skill."""

from copy import copy
import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerResponse

from actionlib import SimpleActionServer
import mobile_manip_demo.msg


class RechargeSkill:
    """Handle the battery management."""

    def __init__(self):
        self.battery_lv = rospy.get_param("moma_demo/battery_lv")
        self.drop_rate = rospy.get_param("moma_demo/battery_drop_rate")
        self.charge_rate = rospy.get_param("moma_demo/battery_charge_rate")
        self.current_lv = copy(self.battery_lv)

        # Action server
        self.action_server = SimpleActionServer(
            "/recharge",
            mobile_manip_demo.msg.RechargeAction,
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
                self.current_lv += self.charge_rate
            else:
                self.current_lv -= self.drop_rate
            self.battery_pub.publish(self.current_lv)
            rate.sleep()

    def execute_callback(self, goal):
        rospy.logwarn("Recharging the batteries!")
        self.recharging = True
        while self.current_lv < self.battery_lv:
            rospy.sleep(2)
        self.recharging = False

        rospy.logwarn("Recharged!")
        self.report_success("Battery recharged!")

    def report_failure(self, msg):
        result = mobile_manip_demo.msg.GraspResult()
        rospy.logerr(msg)
        result.success = False
        result.message = msg
        self.action_server.set_aborted(result)

    def report_preemption(self, msg):
        result = mobile_manip_demo.msg.GraspResult()
        rospy.logwarn(msg)
        result.success = False
        result.message = msg
        self.action_server.set_preempted(result)

    def report_success(self, msg: str):
        result = mobile_manip_demo.msg.GraspResult()
        rospy.loginfo(msg)
        result.success = True
        result.message = msg
        self.action_server.set_succeeded(result)

    def wait_monitoring_preemption(self, client):
        while not client.wait_for_result(timeout=rospy.Duration(1)):
            if self.action_server.is_preempt_requested():
                client.cancel_goal()
                return False
        return True
