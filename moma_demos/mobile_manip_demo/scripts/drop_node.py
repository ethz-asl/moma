#!/usr/bin/env python3

import rospy
from mobile_manip_demo.drop import DropSkill


if __name__ == "__main__":
    rospy.init_node("drop_skill")
    ds = DropSkill()
    rospy.spin()
