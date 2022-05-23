#!/usr/bin/env python3
from typing import List

import actionlib
import rospy
from franka_msgs.msg import ErrorRecoveryAction
from franka_msgs.msg import ErrorRecoveryGoal

rospy.init_node("error_recovery")

client = actionlib.SimpleActionClient(
    "/franka_control/error_recovery", ErrorRecoveryAction
)
rospy.loginfo(f"Waiting for action server...")
client.wait_for_server()
rospy.loginfo(f"... action server found")

goal = ErrorRecoveryGoal()
client.send_goal(goal)
client.wait_for_result()
print(f"result:\n{client.get_result()}")
