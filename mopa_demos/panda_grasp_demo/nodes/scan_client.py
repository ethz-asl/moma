#!/usr/bin/env python

"""Just a small client to test the scan action"""

import rospy
import actionlib

from panda_grasp_demo.msg import ScanSceneAction, ScanSceneGoal 

def feedback_cb(feedback):
    rospy.loginfo("Got feedback: "+str(feedback.percent_complete))

def done_cb(state, result):
    rospy.loginfo("Done")

if __name__ == '__main__':
    rospy.init_node('scanning_client')
    client = actionlib.SimpleActionClient('pointcloud_scan_action', ScanSceneAction)
    client.wait_for_server()

    goal = ScanSceneGoal()
    goal.num_scan_poses = 2
    client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)

    rospy.spin()
    # client.wait_for_result(rospy.Duration.from_sec(30.0))
