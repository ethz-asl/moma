#!/usr/bin/env python
import sys
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
        self.robot_name = sys.argv[1]
        super(SearchActionServer, self).__init__(action_name, SearchAction)
        self._read_waypoints()
        self._read_joint_configurations()
        self._subscribe_object_detection()
        self._connect_robot()

    def _read_waypoints(self):
        waypoints = rospy.get_param("search_waypoints")
        self._initial_waypoints = waypoints["initial"]
        self._loop_waypoints = waypoints["loop"]

    def _read_joint_configurations(self):
        self._robot_arm_names = rospy.get_param("robot_arm_names")
        self._search_joints = {}
        self._ready_joints = {}
        self._home_joints = {}
        arm_purposes = ["grasp", "scan"]
        for i, arm in enumerate(self._robot_arm_names):
            self._search_joints[arm_purposes[i]] = rospy.get_param(
                "search_joints_" + arm, default=None
            )
            self._ready_joints[arm_purposes[i]] = rospy.get_param(
                "ready_joints_" + arm, default=None
            )
            self._home_joints[arm_purposes[i]] = rospy.get_param(
                "home_joints_" + arm, default=None
            )

        self._first_scan_joint = rospy.get_param("scan_joint_values")[0]

    def _subscribe_object_detection(self):
        self._result = None
        self._object_detected = False
        self._subscriber = rospy.Subscriber(
            "/W_landmark", PointStamped, callback=self._object_detection_cb
        )

    def _connect_robot(self):
        self._arm_velocity_scaling = rospy.get_param("arm_velocity_scaling_search")

        full_grasp_arm_name = (
            self.robot_name + "_" + self._robot_arm_names[0]
            if len(self._robot_arm_names) > 1
            else self.robot_name
        )
        self.grasp_arm = create_robot_connection(full_grasp_arm_name)
        if len(self._robot_arm_names) > 1:
            full_scan_arm_name = self.robot_name + "_" + self._robot_arm_names[1]
            self.scan_arm = create_robot_connection(full_scan_arm_name)
        else:
            self.scan_arm = None

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

        if self.scan_arm is not None:
            self.grasp_arm.goto_joint_target(
                self._home_joints["grasp"],
                max_velocity_scaling=self._arm_velocity_scaling,
            )
            self.scan_arm.goto_joint_target(
                self._search_joints["scan"],
                max_velocity_scaling=self._arm_velocity_scaling,
            )
        else:
            self.grasp_arm.goto_joint_target(
                self._search_joints["grasp"],
                max_velocity_scaling=self._arm_velocity_scaling,
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

            # Move arm(s) into the right place for approach.
            # By doing it here, we don't have to move the arms in the approach action.
            if self.scan_arm is not None:
                self.grasp_arm.goto_joint_target(
                    self._ready_joints["grasp"],
                    max_velocity_scaling=self._arm_velocity_scaling,
                )
                self.scan_arm.goto_joint_target(
                    self._first_scan_joint,
                    max_velocity_scaling=self._arm_velocity_scaling,
                )
            else:
                self.grasp_arm.goto_joint_target(
                    self._first_scan_joint,
                    max_velocity_scaling=self._arm_velocity_scaling,
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
    _ = SearchActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
