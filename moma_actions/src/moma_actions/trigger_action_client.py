#! /usr/bin/env python

import rospy
import actionlib
from moma_actions.msg import TriggerGoal, TriggerAction

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus


class TriggerComponentClient:
    def __init__(self) -> None:
        node_name_ = rospy.get_param("~component_node", "/components")
        self._client = actionlib.SimpleActionClient(node_name_, TriggerAction)

        rospy.loginfo(f"Connecting to {node_name_}...")
        self._client.wait_for_server(rospy.Duration(5.0))

    def send_goal(self, component: str, cmd: str) -> None:
        """
        Move the robot with distance direction with type
        """
        goal = TriggerGoal()
        goal.component.data = component
        goal.cmd.data = cmd

        rospy.loginfo(f"Sending goal")
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
        rospy.loginfo("Cancelling the component goal")
        self._client.cancel_goal()
