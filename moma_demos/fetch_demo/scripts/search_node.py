#!/usr/bin/env python

import actionlib
from fetch_demo.msg import SearchAction, SearchResult
import rospy


class SearchActionServer:
    """
        When called, this action should in turn call the navigation action to follow a
        number of pre-defined waypoints.
    """
    def __init__(self):
        action_name = "search_action"
        self.action_server = actionlib.SimpleActionServer(
            action_name, SearchAction, execute_cb=self.search_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Navigation action server started.")

    def search_cb(self, msg):
        rospy.loginfo("Start following search waypoints")
        result = SearchResult()

        for i in range(3):
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Got preempted")
                self.action_server.set_preempted()
                return
            
            # Go to waypoint i using the navigation action
            rospy.sleep(1.0)

        rospy.loginfo("Reached last waypoint")
        self.action_server.set_succeeded(result)

def main():
    rospy.init_node("search_action_node")
    action = SearchActionServer()
    rospy.spin()

if __name__ == "__main__":
    main()
