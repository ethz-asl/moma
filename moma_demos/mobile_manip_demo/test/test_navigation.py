#!/usr/bin/env python

from geometry_msgs.msg import Pose
from mobile_manip_demo.robot_interface import Move, RobotAtPose

import rospy
import tf2_ros

import numpy as np


class NavigationNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        self.waypoints = [
            [4.5, 0.0, 0.0],
            [4.5, -3.0, 0.0],
            [0.0, -3.0, 0.0],
            [0.0, 0.0, 0.0],
        ]

        namespace = "/mobile_base/"
        self.move_action = Move(namespace)
        self.at_pose = RobotAtPose("panda", namespace)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def follow_waypoints(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Transmitting waypoints.")
            for point in self.waypoints:
                rospy.loginfo("Executing point " + str(point))
                self.move_action.cancel_goal()
                goal_pose = Pose()
                goal_pose.position.x = point[0]
                goal_pose.position.y = point[1]
                goal_pose.position.z = point[2]
                goal_pose.orientation.x = 0.0
                goal_pose.orientation.y = 0.0
                goal_pose.orientation.z = 0.0
                goal_pose.orientation.w = 1.0

                self.move_action.initialize_navigation(goal_pose)

                move_base_done = False
                while not move_base_done:
                    status = self.move_action.get_navigation_status()
                    if status == 0 or status == 1:
                        rospy.loginfo("move base RUNNING")
                    elif status == 3:
                        rospy.loginfo("move base SUCCESS")
                        move_base_done = True
                    else:
                        rospy.loginfo(str(status))
                        rospy.loginfo("move base FAILURE")
                        move_base_done = True

                # condition checking
                amcl_pose_ok = self.at_pose.at_pose(
                    target_pose=np.array(point), tolerance=0.5
                )
                rospy.loginfo("Pose reached (AMCL)?: " + str(amcl_pose_ok))
                # with ground truth = robot pose in gazebo
                model_pose_ok = self.at_pose.at_pose(
                    target_pose=np.array(point), tolerance=0.5, ground_truth=True
                )
                rospy.loginfo("Pose reached (gazebo)?: " + str(model_pose_ok))


def main():
    rospy.init_node("navigation_tester_node")
    node = NavigationNode()

    try:
        node.follow_waypoints()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
