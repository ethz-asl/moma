#!/usr/bin/env python
import actionlib
from fetch_demo.msg import SearchAction, SearchResult
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fetch_demo.approach_node import MovingAction


class SearchActionServer(MovingActionServer):
    """
        When called, this action should in turn call the navigation action to follow a
        number of pre-defined waypoints. Simultaneously check the object detection if
        the object was found.
    """

    def __init__(self):
        action_name = "search_action"
        super(SearchActionServer, self).__init__(action_name, SearchAction)
        self._read_waypoints()

    def _read_waypoints(self):
        waypoints = rospy.get_param("search_waypoints")
        self._initial_waypoints = waypoints["initial"]
        self._loop_waypoints = waypoints["loop"]

    def action_callback(self, msg):
        rospy.loginfo("Start following search waypoints")
        result = SearchResult()

        for waypoint in self._initial_waypoints:
            if not self._visit_waypoint(waypoint):
                rospy.logerr("Failed to reach waypoint {}".format(waypoint))
                return

        while True:
            # TODO: Stop iterating the waypoints when the object is detected.
            for waypoint in self._loop_waypoints:
                if not self._visit_waypoint(waypoint):
                    rospy.logerr("Failed to reach waypoint {}".format(waypoint))
                    return

        self.action_server.set_succeeded(result)


def main():
    rospy.init_node("search_action_node")
    action = SearchActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
