#!/usr/bin/env python
import actionlib
from fetch_demo.msg import SearchAction, SearchResult
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SearchActionServer:
    """
        When called, this action should in turn call the navigation action to follow a
        number of pre-defined waypoints. Simultaneously check the object detection if
        the object was found.
    """

    def __init__(self):
        action_name = "search_action"
        self._read_waypoints()
        self.action_server = actionlib.SimpleActionServer(
            action_name, SearchAction, execute_cb=self.search_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Search action server started.")
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

    def _read_waypoints(self):
        waypoints = rospy.get_param("search_waypoints")
        self._initial_waypoints = waypoints["initial"]
        self._loop_waypoints = waypoints["loop"]

    def search_cb(self, msg):
        rospy.loginfo("Start following search waypoints")
        result = SearchResult()

        for waypoint in self._initial_waypoints:
            if not self._visit_waypoint(waypoint):
                return

        while True:
            # TODO: Stop iterating the waypoints when the object is detected.
            for waypoint in self._loop_waypoints:
                if not self._visit_waypoint(waypoint):
                    return

        self.action_server.set_succeeded(result)

    def _visit_waypoint(self, waypoint):
        """ Calls move_base to visit the given waypoint.
            returns true if the moving succeeded, false otherwise. """
        if self.action_server.is_preempt_requested():
            rospy.loginfo("Search action got pre-empted")
            self.action_server.set_preempted()
            return False
        pose = self._waypoint_to_pose_msg(waypoint)
        navigation_goal = MoveBaseGoal(target_pose=pose)
        self.move_base_client.send_goal(navigation_goal)
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()
        if state is not GoalStatus.SUCCEEDED:
            rospy.error("Failed to reached waypoint {}".format(waypoint))
            self.action_server.set_aborted()
            return False

        rospy.info("Reached waypoint {}".format(waypoint))
        rospy.sleep(0.5)
        return True

    def _waypoint_to_pose_msg(self, waypoint):
        orn = Rotation.from_euler("z", np.deg2rad(waypoints[2])).as_quat()
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.x = waypoint[0]
        msg.pose.y = waypoint[1]
        msg.pose.z = 0.0
        msg.orientation.x = orn[0]
        msg.orientation.y = orn[1]
        msg.orientation.z = orn[2]
        msg.orientation.w = orn[3]
        return msg


def main():
    rospy.init_node("search_action_node")
    action = SearchActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
