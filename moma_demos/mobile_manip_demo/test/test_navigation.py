#!/usr/bin/env python

from typing import Any, List
from geometry_msgs.msg import Pose
from mobile_manip_demo.robot_interface import Move, RobotAtPose, Search

import rospy
import tf2_ros

import numpy as np


class NavigationNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        self.waypoints = rospy.get_param("moma_demo/search_waypoints")

        self.move_action = Move()
        self.at_pose = RobotAtPose("panda")

        self.search_action = Search()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def simple_goal(self):
        point = self.waypoints[0]
        rospy.loginfo("Sending goal." + str(point))

        goal_pose = Pose()
        goal_pose.position.x = point[0]
        goal_pose.position.y = point[1]
        goal_pose.position.z = point[2]
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0

        self.move_action.cancel_goal()
        self.move_action.initialize_navigation(goal_pose)

        self.check_feedback(self.move_action, point)

    def goal_from_ID(self):
        self.move_action.cancel_goal()
        self.move_action.initialize_navigation(goal_ID=2)

        self.check_feedback(self.move_action, 2)

    def follow_waypoints(self):
        rospy.loginfo("Transmitting waypoints.")
        for point in self.waypoints:
            rospy.loginfo("Executing point " + str(point))
            self.search_action.cancel_goal()
            self.search_action.initialize_search(self.waypoints)

            self.check_feedback(self.search_action, point)

    def check_feedback(self, client: Any, point: List[float] or int):
        if type(point) == list:
            point = np.array(point)[:3]
        move_base_done = False
        while not move_base_done:
            try:
                status = client.get_search_status()
            except AttributeError:
                status = client.get_navigation_status()
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
        amcl_pose_ok = self.at_pose.at_pose(target_pose=point, tolerance=0.5)
        rospy.loginfo("Pose reached (AMCL)?: " + str(amcl_pose_ok))
        # with ground truth = robot pose in gazebo
        model_pose_ok = self.at_pose.at_pose(
            target_pose=point, tolerance=0.5, ground_truth=True
        )
        rospy.loginfo("Pose reached (gazebo)?: " + str(model_pose_ok))


def main():
    rospy.init_node("navigation_tester_node")
    node = NavigationNode()

    try:
        # node.simple_goal()
        node.goal_from_ID()
        # node.follow_waypoints()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
