#!/usr/bin/env python
from panda_control.panda_commander import PandaCommander
import rospy

rospy.init_node("jlfkjaskldfjlk")
pc = PandaCommander()

rospy.loginfo("Got a commander")

while True:
    res = raw_input("a: close, b: open")
    if res == "a":
        pc.grasp()
    elif res == "b":
        pc.release()
    elif res == "s":
        pc.stop()
    else:
        break
