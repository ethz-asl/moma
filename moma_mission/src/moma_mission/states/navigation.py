#!/usr/bin/env python

import tf
import math
import numpy as np
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

from moma_mission.core import StateRosControl


class SingleNavGoalState(StateRosControl):
    """
    In this state the robot navigates to a single goal"""

    def __init__(self, ns="", outcomes=['Completed', 'Failure']):
        StateRosControl.__init__(self, ns=ns, outcomes=outcomes)

        self.goal_action_topic = self.get_scoped_param("goal_action_topic")
        self.goal_pose_topic = self.get_scoped_param("goal_pose_topic")
        self.base_pose_topic = self.get_scoped_param("base_pose_topic")
        self.goal_publisher = rospy.Publisher(self.goal_pose_topic, PoseStamped, queue_size=10)
        self.base_pose_subscriber = rospy.Subscriber(self.base_pose_topic, PoseStamped, self.base_pose_callback)

        self.timeout = self.get_scoped_param("timeout")
        self.tolerance_m = self.get_scoped_param("tolerance_m")
        self.tolerance_rad = self.get_scoped_param("tolerance_deg") * math.pi / 180.0

        self.goal = None
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw_rad = 0.0

        self.base_pose_received = False

    def reach_goal(self, goal, action=False):
        if not isinstance(goal, PoseStamped):
            rospy.logerr("The goal needs to be specified as a PoseStamped message")
            return False

        controller_switched = self.do_switch()
        if not controller_switched:
            return False

        if action:
            return self._reach_via_action(goal)
        else:
            return self._reach_via_topic(goal)

    def _reach_via_action(self, goal):
        goal_client = actionlib.SimpleActionClient(self.goal_action_topic, MoveBaseAction)

        # Waits until the action server has started up and started
        # listening for goals.
        if not goal_client.wait_for_server(timeout=rospy.Duration(3.0)):
            rospy.logerr("Failed to contact server at [{}]".format(self.goal_action_topic))
        
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose = goal
        goal_msg.header = goal.header

        # Sends the goal to the action server.
        goal_client.send_goal(goal_msg.goal)
        success = goal_client.wait_for_result(timeout=rospy.Duration(10*60))
        return success

    def _reach_via_topic(self, goal):
        while not self.goal_publisher.get_num_connections():
            rospy.loginfo_throttle(1.0, "Waiting for subscriber to connect to waypoint topic")

        self.goal_publisher.publish(goal)
        self.goal = goal
        start_time = rospy.get_rostime().to_sec()
        elapsed_time = 0
        while not rospy.is_shutdown():
            if self.reached_waypoint_with_tolerance(goal) and elapsed_time < self.timeout:
                rospy.loginfo("Goal reached")
                return True
            elif elapsed_time >= self.timeout:
                rospy.logerr("Timeout reached while reaching the goal")
                return False
            else:
                rospy.loginfo_throttle(3.0, "Reaching the goal ... {} s to timeout".format(self.timeout - elapsed_time))
                rospy.sleep(1.0)
            elapsed_time = rospy.get_rostime().to_sec() - start_time

    def run(self):
        raise NotImplementedError("This function needs to be implemented")

    def base_pose_callback(self, pose_stamped_msg):
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        self.base_x = x_m
        self.base_y = y_m
        self.base_yaw_rad = yaw_rad
        self.base_pose_received = True

    def reached_waypoint_with_tolerance(self, goal):
        lin_tol_ok = False
        ang_tol_ok = False
        if self.base_pose_received:
            distance_to_waypoint = math.sqrt(pow(goal.pose.position.x - self.base_x, 2) +
                                             pow(goal.pose.position.y - self.base_y, 2))

            _, _, goal_yaw = tf.transformations.euler_from_quaternion([goal.pose.orientation.x,
                                                                       goal.pose.orientation.y,
                                                                       goal.pose.orientation.z,
                                                                       goal.pose.orientation.w])
            rospy.loginfo(f"""
angle to waypoint:    {np.rad2deg(angle_to_waypoint)}, tolerance: {np.rad2deg(self.tolerance_deg)}
distance to waypoint: {distance_to_waypoint}, tolerance: {self.tolerance_m}
""")
            angle_to_waypoint = abs(self.base_yaw_rad - goal_yaw)
            lin_tol_ok = (distance_to_waypoint <= self.tolerance_m)
            ang_tol_ok = (angle_to_waypoint <= self.tolerance_rad)
        return lin_tol_ok and ang_tol_ok and self.base_pose_received
