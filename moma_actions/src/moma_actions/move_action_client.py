#! /usr/bin/env python

import rospy
import actionlib
from moma_actions.msg import MoveAction, MoveGoal, MoveFeedback

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

class MoveClient:
    def __init__(self) -> None:
        node_name_ = rospy.get_param("~move_node", "/move")
        self._client = actionlib.SimpleActionClient(node_name_, MoveAction)

        rospy.loginfo(f"Connecting to {node_name_}...")
        self._client.wait_for_server(rospy.Duration(5.0))

    def send_distance_goal(self, direction: str, distance: float) -> None:
        """
        Move the robot with distance direction with type
        """
        goal = MoveGoal()
        goal.direction.data = direction
        goal.distance = distance

        rospy.loginfo(f"Sending goal")
        self._client.send_goal(goal)

    def send_combined_goal(self, goal_pose: Pose, ref_frame: str = "map") -> None:
        """
        Move the robot to a target pose.
        """
        goal = MoveGoal()
        goal.direction.data = 'combined'
        print(f"frame_id: {goal.target_pose.header.frame_id}")
        print(f"time: {goal.target_pose.header.stamp}")
        print(f"pose: {goal.target_pose.pose}")

        goal.target_pose.header.frame_id = ref_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        rospy.loginfo(f"Sending combined goal")
        self._client.send_goal(goal)

    def get_status(self) -> int:
        """
        get move_base status
        https://docs.ros.org/en/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
        """
        if self._client.get_state() == GoalStatus.SUCCEEDED:
            rospy.logwarn("Successfully reached the goal")
            return GoalStatus.SUCCEEDED
        else:
            return self._client.get_state()

    def cancel_goal(self) -> None:
        """
        Cancel the current goal
        """
        rospy.loginfo("Cancelling the move goal")
        self._client.cancel_goal()
