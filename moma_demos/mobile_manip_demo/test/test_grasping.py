#!/usr/bin/env python

from geometry_msgs.msg import Pose
from mobile_manip_demo.robot_interface import Pick, ObjectAtPose

import rospy
import tf2_ros
import tf2_geometry_msgs

import numpy as np


class GraspingNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        self.with_id = True

        self.pick_action = Pick()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def send_pick_request(self):
        self.pick_action.cancel_goal()

        done = False
        attempts = 0
        while not done or attempts > 15:
            try:
                msg = self.tf_buffer.lookup_transform(
                    "task",
                    "tag_2",
                    rospy.Time(),
                    rospy.Duration(5),
                )
                done = True
            except Exception:
                attempts += 1
                rospy.logerr("Could not get transform, retrying...")

        goal = Pose()
        goal.position.x = msg.transform.translation.x
        goal.position.y = msg.transform.translation.y
        goal.position.z = msg.transform.translation.z
        goal.orientation = msg.transform.rotation

        if self.with_id:
            self.pick_action.initialize_pick(goal_ID=2)
        else:
            self.pick_action.initialize_pick(goal, ref_frame="task")
            rospy.loginfo(f"Sending goal:\n {goal.position}.")

        pick_done = False
        while not pick_done:
            status = self.pick_action.get_pick_status()
            if status == 0 or status == 1:
                rospy.loginfo("pick RUNNING")
            elif status == 3:
                rospy.loginfo("pick SUCCESS")
                pick_done = True
            else:
                rospy.loginfo(str(status))
                rospy.loginfo("pick FAILURE")
                pick_done = True


def main():
    rospy.init_node("grasping_tester_node")
    node = GraspingNode()

    try:
        node.send_pick_request()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
