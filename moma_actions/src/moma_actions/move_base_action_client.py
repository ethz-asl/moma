#!/usr/bin/env python

from __future__ import annotations  # for type hinting

import rospy
import actionlib

from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class MoveBaseClient:
    def __init__(self) -> None:
        node_name_ = rospy.get_param(
            "~move_base_node", "/mobile_base/move_base"
        )
        self.move_base_client = actionlib.SimpleActionClient(
            node_name_, MoveBaseAction
        )
        rospy.loginfo(f"Connecting to {node_name_}...")
        self.move_base_client.wait_for_server(rospy.Duration(5.0))

    def init_move_base(self, goal_pose: Pose, ref_frame: str = "map") -> None:
        """
        Move the robot to a target pose.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = ref_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        # send the goal
        rospy.loginfo(f"Sending goal to move_base")
        self.move_base_client.send_goal(goal)

    def get_status(self) -> int:
        """
        get move_base status
        https://docs.ros.org/en/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
        """
        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            return GoalStatus.SUCCEEDED
        else:
            return self.move_base_client.get_state()

    def cancel_goal(self) -> None:
        """
        Cancel the current goal
        """
        rospy.loginfo("Cancelling move_base goal")
        self.move_base_client.cancel_goal()
