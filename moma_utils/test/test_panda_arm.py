#!/usr/bin/env python

from __future__ import print_function

import rospy

from moma_utils.ros.panda import PandaArmClient


def main():
    rospy.init_node("test_panda_arn")

    arm = PandaArmClient()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if arm.has_error:
            arm.recover()
        q, _ = arm.get_state()
        print("Current joint positions: ", q)
        r.sleep()


if __name__ == "__main__":
    main()
