#!/usr/bin/env python
import actionlib
from actionlib_msgs.msg import GoalStatus
from fetch_demo.msg import SearchAction, SearchResult
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fetch_demo.common import MovingActionServer

from grasp_demo.utils import create_robot_connection


class SearchDoneException(Exception):
    # Thrown when search needs to be stopped
    # for whatever reason (abort, pre-emption, completion).
    pass


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
        self._read_joint_configurations()
        self._subscribe_object_detection()
        self._connect_yumi()

    def _read_waypoints(self):
        waypoints = rospy.get_param("search_waypoints")
        self._initial_waypoints = waypoints["initial"]
        self._loop_waypoints = waypoints["loop"]

    def _read_joint_configurations(self):
        self._search_joints_r = rospy.get_param("search_joints_r")
        self._ready_joints_l = rospy.get_param("ready_joints_l")
        self._home_joints_l = rospy.get_param("home_joints_l")
        self._first_scan_joint_r = rospy.get_param("scan_joint_values")[0]

    def _subscribe_object_detection(self):
        self._result = None
        self._object_detected = False
        self._subscriber = rospy.Subscriber(
            "/W_landmark", PointStamped, callback=self._object_detection_cb
        )

    def _connect_yumi(self):
        self.left_arm = create_robot_connection("yumi_left_arm")
        self.right_arm = create_robot_connection("yumi_right_arm")

    def _object_detection_cb(self, msg):
        self._object_detected = True
        self._result = SearchResult()
        self._result.target_object_pose.position.x = msg.point.x
        self._result.target_object_pose.position.y = msg.point.y
        self._result.target_object_pose.position.z = 0.0
        rospy.loginfo("Object detection message received. Stopping search.")
        self.move_base_client.cancel_all_goals()

    def action_callback(self, msg):
        self._object_detected = False
        rospy.loginfo("Start following search waypoints")

        self.left_arm.goto_joint_target(self._home_joints_l, max_velocity_scaling=0.5)
        self.right_arm.goto_joint_target(
            self._search_joints_r, max_velocity_scaling=0.5
        )

        try:
            for waypoint in self._initial_waypoints:
                self._handle_waypoint(waypoint)

            while True:
                for waypoint in self._loop_waypoints:
                    self._handle_waypoint(waypoint)
        except SearchDoneException:
            return

    def _handle_waypoint(self, waypoint):
        state = self._visit_waypoint(waypoint)
        if self._object_detected:
            rospy.loginfo("Search completed")
            self.right_arm.goto_joint_target(
                self._first_scan_joint_r, max_velocity_scaling=0.5
            )
            self.left_arm.goto_joint_target(
                self._ready_joints_l, max_velocity_scaling=0.5
            )
            self.action_server.set_succeeded(self._result)
            raise SearchDoneException()
        elif state == GoalStatus.PREEMPTED:
            rospy.loginfo("Got preemption request")
            self.action_server.set_preempted()
            raise SearchDoneException()
        elif state == GoalStatus.ABORTED:
            rospy.logerr("Failed to navigate to approach waypoint")
            self.action_server.set_aborted()
            raise SearchDoneException()


def main():
    rospy.init_node("search_action_node")
    action = SearchActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
