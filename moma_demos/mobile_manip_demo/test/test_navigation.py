#!/usr/bin/env python

from typing import Any

from geometry_msgs.msg import Pose

# Action lib stuff
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy
import tf2_ros


class Move:
    """Low level implementation of a Navigation skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.move_client = SimpleActionClient("/mobile_base/move_base", MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(100))

    def initialize_navigation(
        self,
        goal_pose: Pose = Pose(),
        ref_frame: str = "map",
        goal_ID: str = None,
        goal_register: Any = None,
    ) -> None:
        """
        Move the robot to the target pose.

        Args:
        ----
            - goal_pose: if desired, set directly the goal to send.
            - ref_frame: reference frame for the goal.
            - goal_ID: a string ID for the goal. If given, also goal_register must be provided.
            - goal_register: function linking goal_ID to an actual goal expressed as Pose().

        """
        if goal_ID is not None:
            target_goal, ref_frame = goal_register(goal_ID)
        # command
        goal_ = MoveBaseGoal()
        goal_.target_pose.header.frame_id = ref_frame
        goal_.target_pose.header.stamp = rospy.Time.now()
        goal_.target_pose.pose = target_goal if goal_ID is not None else goal_pose

        # send the goal
        self.move_client.send_goal(goal_)

    def get_navigation_status(self) -> int:
        """
        Get result from navigation.

        uint8 PENDING         = 0   # The goal has yet to be processed by the action server
        uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
        uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                    #   and has since completed its execution (Terminal State)
        uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
        uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                    #    to some failure (Terminal State)
        uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                    #    because the goal was unattainable or invalid (Terminal State)
        uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                    #    and has not yet completed execution
        uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                    #    but the action server has not yet confirmed that the goal is canceled
        uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                    #    and was successfully cancelled (Terminal State)
        uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                    #    sent over the wire by an action server
        """
        wait = self.move_client.wait_for_result(rospy.Duration(60))
        if not wait:
            rospy.logerr("Move Action server not available!")
            rospy.signal_shutdown("Move Action server not available!")
            return -1
        else:
            # Result of executing the action
            return self.move_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        self.move_client.cancel_goal()


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

        self.move_action = Move()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def follow_waypoints(self):
        while not rospy.is_shutdown():
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

                done = False
                while not done:
                    status = self.move_action.get_navigation_status()
                    if status == 0 or status == 1:
                        rospy.loginfo("RUNNING")
                    elif status == 3:
                        rospy.loginfo("SUCCESS")
                        done = True
                    else:
                        rospy.loginfo(str(status))
                        rospy.loginfo("FAILURE")
                        done = True


def main():
    rospy.init_node("navigation_tester_node")
    node = NavigationNode()

    try:
        node.follow_waypoints()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
