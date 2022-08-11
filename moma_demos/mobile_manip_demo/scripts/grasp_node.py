#!/usr/bin/env python3

import rospy
from mobile_manip_demo.grasp import GraspSkill


if __name__ == "__main__":
    rospy.init_node("grasp_skill")
    gs = GraspSkill()
    rospy.spin()
