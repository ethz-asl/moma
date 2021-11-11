#!/usr/bin/env python
import rospy
from moma_utils.ros.panda import PandaGripperClient

rospy.init_node("jlfkjaskldfjlk")
gc = PandaGripperClient()

rospy.loginfo("Got a commander")
gc.home()

while True:
    res = raw_input("a: close, b: open")
    if res == "a":
        gc.grasp2()
    elif res == "b":
        # gc.release()
        gc.move(0.1)
    elif res == "s":
        gc.stop()
    else:
        break
