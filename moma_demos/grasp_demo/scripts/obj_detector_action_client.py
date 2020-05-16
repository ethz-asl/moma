#! /usr/bin/env python

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import grasp_demo.msg

def detection_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('CVdetector', actionlib_tutorials.msg.DetectionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.ActionGoal(name = "person")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A DetectionResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('detection_client_py')
        result = detection_client()
        print("Result:", result.targetBB)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
