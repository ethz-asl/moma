#!/usr/bin/env python
from panda_control.panda_commander import PandaCommander
import rospy

rospy.init_node("jlfkjaskldfjlk")
pc = PandaCommander()

rospy.loginfo("Got a commander")

raw_input("Press enter to continue")

pc.grasp()

raw_input("Press enter to continue")

pc.release()
