#!/usr/bin/env python

import actionlib
from fetch_demo.msg import (
    ApproachAction,
    ApproachResult,
    NavigationAction,
    NavigationResult,
    SearchAction,
    SearchResult,
)
from geometry_msgs.msg import PoseStamped
import rospy


class DummyActionServers:
    def __init__(self):
        self.action_server_approach = actionlib.SimpleActionServer(
            "approach_action",
            ApproachAction,
            execute_cb=self.approach_cb,
            auto_start=False,
        )
        self.action_server_approach.start()
        rospy.loginfo("Approach action server running.")

        self.action_server_navigation = actionlib.SimpleActionServer(
            "navigation_action",
            NavigationAction,
            execute_cb=self.navigation_cb,
            auto_start=False,
        )
        self.action_server_navigation.start()
        rospy.loginfo("Navigation action server running.")

        self.action_server_search = actionlib.SimpleActionServer(
            "search_action", SearchAction, execute_cb=self.search_cb, auto_start=False
        )
        self.action_server_search.start()
        rospy.loginfo("Search action server running.")

    def approach_cb(self, goal):
        rospy.loginfo("Approach action was triggered")
        result = ApproachResult()
        rospy.sleep(3.0)
        rospy.loginfo("Success")
        self.action_server_approach.set_succeeded()

    def navigation_cb(self, goal):
        rospy.loginfo("Navigation action was triggered")
        result = NavigationResult()
        rospy.sleep(1.0)
        rospy.loginfo("Success")
        self.action_server_navigation.set_succeeded(result)

    def search_cb(self, goal):
        rospy.loginfo("Search action was triggered")
        result = SearchResult()
        rospy.sleep(1.0)
        rospy.loginfo("Success")
        self.action_server_search.set_succeeded(result)


def main():
    rospy.init_node("fetch_demo_dummy_actions_node")

    das = DummyActionServers()

    rospy.spin()


if __name__ == "__main__":
    main()
