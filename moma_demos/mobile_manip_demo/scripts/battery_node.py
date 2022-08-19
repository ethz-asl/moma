#!/usr/bin/env python3

import rospy
from mobile_manip_demo.recharge import RechargeSkill


if __name__ == "__main__":
    rospy.init_node("recharge_skill")
    rs = RechargeSkill()
    rs.publish_battery_lv()
    rospy.spin()
