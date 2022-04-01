#!/usr/bin/env python3
from typing import List
import rospy
import actionlib
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal

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
