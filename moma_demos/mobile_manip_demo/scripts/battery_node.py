#!/usr/bin/env python3

from copy import copy
import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerResponse


class BatteryManagement:
    """Handle the battery management."""

    def __init__(self):
        self.battery_lv = rospy.get_param("moma_demo/battery_lv")
        self.drop_rate = rospy.get_param("moma_demo/battery_drop_rate")
        self.charge_rate = rospy.get_param("moma_demo/battery_charge_rate")
        self.current_lv = copy(self.battery_lv)

        self.battery_pub = rospy.Publisher("/battery_level", Int32, queue_size=10)

        self.recharge_srv = rospy.Service("/recharge", Trigger, self.recharge)
        self.recharging = False

    def publish_battery_lv(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.recharging:
                self.current_lv += self.charge_rate
            else:
                self.current_lv -= self.drop_rate
            self.battery_pub.publish(self.current_lv)
            rate.sleep()

    def recharge(self, req):
        rospy.logwarn("Recharging the batteries!")
        self.recharging = True
        while self.current_lv < self.battery_lv:
            rospy.sleep(2)
        self.recharging = False

        rospy.logwarn("Recharged!")
        response = TriggerResponse()
        response.success = True
        response.message = ""
        return response


if __name__ == "__main__":
    rospy.init_node("battery_manager")
    bm = BatteryManagement()
    bm.publish_battery_lv()
