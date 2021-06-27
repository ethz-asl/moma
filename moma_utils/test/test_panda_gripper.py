#!/usr/bin/env python

from __future__ import print_function

import rospy

from moma_utils.ros.panda import PandaGripperClient


def main():
    rospy.init_node("test_panda_gripper")
    gripper = PandaGripperClient()
    while True:
        res = raw_input("Enter command (g: grasp, r: release, s: stop): ")
        if res == "g":
            gripper.grasp()
        elif res == "r":
            gripper.release()
        elif res == "s":
            gripper.stop()
        else:
            break
        print("Current width:", gripper.read())


if __name__ == "__main__":
    main()
