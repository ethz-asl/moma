#!/usr/bin/env python3
import math

import actionlib
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moma_mission.core import StateRos
from moma_mission.core import StateRosControl
from moma_mission.utils import ros
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal


class WaypointNavigation(StateRosControl):
    """
    In this state the robot navigates through a sequence of waypoint which are sent to the
    global planner once the previous has been reached within a certain tolerance
    """

    def __init__(self, mission, waypoint_pose_topic, base_pose_topic, ns=""):
        StateRosControl.__init__(
            self, outcomes=["Completed", "Failure", "Next Waypoint"], ns=ns
        )
        self.mission_data = mission
        self.waypoint_idx = 0

        self.waypoint_pose_publisher = rospy.Publisher(
            waypoint_pose_topic, PoseStamped, queue_size=10
        )
        self.base_pose_subscriber = rospy.Subscriber(
            base_pose_topic, PoseStamped, self.base_pose_callback
        )

        self.countdown_s = 60
        self.countdown_decrement_s = 1
        self.distance_to_waypoint_tolerance_m = 0.3
        self.angle_to_waypoint_tolerance_rad = 0.7

        self.waypoint_x_m = 0.0
        self.waypoint_y_m = 0.0
        self.waypoint_yaw_rad = 0.0

        self.base_pose_received = False
        self.estimated_x_m = 0.0
        self.estimated_y_m = 0.0
        self.estimated_yaw_rad = 0.0

    @staticmethod
    def read_missions_data(mission_file):
        """
        Reads the mission data and return the corresponding dictionary
        :param mission_file:
        :return:
        """
        assert mission_file.endswith(".yaml")
        with open(mission_file, "r") as stream:
            return yaml.load(stream)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        if self.waypoint_idx >= len(self.mission_data.keys()):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return "Completed"

        success = rocoma.switch_roco_controller(
            "MpcTrackLocalPlan", ns="/smb_highlevel_controller"
        )
        if not success:
            rospy.logerr("Could not execute the navigation plan")
            return "Aborted"

        current_waypoint_name = self.mission_data.keys()[self.waypoint_idx]
        current_waypoint = self.mission_data[current_waypoint_name]

        self.set_waypoint(
            current_waypoint["x_m"],
            current_waypoint["y_m"],
            current_waypoint["yaw_rad"],
        )
        rospy.loginfo("Waypoint set: '" + current_waypoint_name + "'.")

        countdown_s = self.countdown_s
        while countdown_s and not rospy.is_shutdown():
            if self.reached_waypoint_with_tolerance():
                rospy.loginfo(
                    "Waypoint '"
                    + current_waypoint_name
                    + "' reached before countdown ended. Loading next waypoint..."
                )
                self.waypoint_idx += 1
                return "Next Waypoint"
            else:
                rospy.loginfo_throttle(
                    5.0,
                    str(countdown_s)
                    + "s left until skipping waypoint '"
                    + current_waypoint_name
                    + "'.",
                )
                rospy.sleep(self.countdown_decrement_s)
            countdown_s -= self.countdown_decrement_s
        rospy.logwarn(
            "Countdown ended without reaching waypoint '" + current_waypoint_name + "'."
        )
        if self.waypoint_idx == 0:
            rospy.logwarn(
                "Starting waypoint of mission unreachable. Aborting current mission."
            )
            self.waypoint_idx = 0.0
            return "Aborted"
        else:
            rospy.logwarn("Skipping waypoint '" + current_waypoint_name + "'.")
            self.waypoint_idx += 1
            return "Next Waypoint"

    def set_waypoint(self, x_m, y_m, yaw_rad):
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.seq = 0
        pose_stamped_msg.header.stamp.secs = rospy.get_rostime().secs
        pose_stamped_msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        pose_stamped_msg.header.frame_id = "world"
        pose_stamped_msg.pose.position.x = x_m
        pose_stamped_msg.pose.position.y = y_m
        pose_stamped_msg.pose.position.z = 0.0
        pose_stamped_msg.pose.orientation.x = quaternion[0]
        pose_stamped_msg.pose.orientation.y = quaternion[1]
        pose_stamped_msg.pose.orientation.z = quaternion[2]
        pose_stamped_msg.pose.orientation.w = quaternion[3]

        while not self.waypoint_pose_publisher.get_num_connections():
            rospy.loginfo_throttle(
                1.0, "Waiting for subscriber to connect to waypoint topic"
            )
        self.waypoint_pose_publisher.publish(pose_stamped_msg)

        self.waypoint_x_m = x_m
        self.waypoint_y_m = y_m
        self.waypoint_yaw_rad = yaw_rad

    def base_pose_callback(self, pose_stamped_msg):
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(
            explicit_quat
        )

        self.estimated_x_m = x_m
        self.estimated_y_m = y_m
        self.estimated_yaw_rad = yaw_rad
        self.base_pose_received = True

    def reached_waypoint_with_tolerance(self):
        try:
            lin_tol_ok = False
            ang_tol_ok = False
            if self.base_pose_received:
                distance_to_waypoint = math.sqrt(
                    pow(self.waypoint_x_m - self.estimated_x_m, 2)
                    + pow(self.waypoint_y_m - self.estimated_y_m, 2)
                )
                angle_to_waypoint = abs(self.waypoint_yaw_rad - self.estimated_yaw_rad)
                lin_tol_ok = (
                    distance_to_waypoint <= self.distance_to_waypoint_tolerance_m
                )
                ang_tol_ok = angle_to_waypoint <= self.angle_to_waypoint_tolerance_rad
            return lin_tol_ok and ang_tol_ok and self.base_pose_received
        except:
            rospy.logwarn("No estimated base pose received yet.")
            return False


class SingleNavGoalState(StateRosControl):
    """
    In this state the robot navigates to a single goal"""

    def __init__(self, ns="", outcomes=["Completed", "Failure"]):
        StateRosControl.__init__(self, ns=ns, outcomes=outcomes)

        self.goal_action_topic = self.get_scoped_param("goal_action_topic")
        self.goal_pose_topic = self.get_scoped_param("goal_pose_topic")
        self.base_pose_topic = self.get_scoped_param("base_pose_topic")
        self.goal_publisher = rospy.Publisher(
            self.goal_pose_topic, PoseStamped, queue_size=10
        )
        self.base_pose_subscriber = rospy.Subscriber(
            self.base_pose_topic, PoseStamped, self.base_pose_callback
        )

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
        goal_client = actionlib.SimpleActionClient(
            self.goal_action_topic, MoveBaseAction
        )

        # Waits until the action server has started up and started
        # listening for goals.
        if not goal_client.wait_for_server(timeout=rospy.Duration(3.0)):
            rospy.logerr(
                "Failed to contact server at [{}]".format(self.goal_action_topic)
            )

        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose = goal
        goal_msg.header = goal.header

        # Sends the goal to the action server.
        goal_state = goal_client.send_goal_and_wait(
            goal_msg.goal, execute_timeout=rospy.Duration(10 * 60)
        )
        rospy.loginfo(f"Move base returned {goal_state}")
        return goal_state == GoalStatus.SUCCEEDED

    def _reach_via_topic(self, goal):
        while not self.goal_publisher.get_num_connections():
            rospy.loginfo_throttle(
                1.0, "Waiting for subscriber to connect to waypoint topic"
            )

        self.goal_publisher.publish(goal)
        self.goal = goal
        start_time = rospy.get_rostime().to_sec()
        elapsed_time = 0
        while not rospy.is_shutdown():
            if self.reached_waypoint_with_tolerance() and elapsed_time < self.timeout:
                rospy.loginfo("Goal reached")
                return True
            elif elapsed_time >= self.timeout:
                rospy.logerr("Timeout reached while reaching the goal")
                return False
            else:
                rospy.loginfo_throttle(
                    3.0,
                    "Reaching the goal ... {} s to timeout".format(
                        self.timeout - elapsed_time
                    ),
                )
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
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(
            explicit_quat
        )

        self.base_x = x_m
        self.base_y = y_m
        self.base_yaw_rad = yaw_rad
        self.base_pose_received = True

    def reached_waypoint_with_tolerance(self):
        lin_tol_ok = False
        ang_tol_ok = False
        if self.base_pose_received:
            distance_to_waypoint = math.sqrt(
                pow(self.goal.pose.position.x - self.base_x, 2)
                + pow(self.goal.pose.position.y - self.base_y, 2)
            )

            _, _, goal_yaw = tf.transformations.euler_from_quaternion(
                [
                    self.goal.pose.orientation.x,
                    self.goal.pose.orientation.y,
                    self.goal.pose.orientation.z,
                    self.goal.pose.orientation.w,
                ]
            )
            angle_to_waypoint = abs(self.base_yaw_rad - goal_yaw)
            lin_tol_ok = distance_to_waypoint <= self.tolerance_m
            ang_tol_ok = angle_to_waypoint <= self.tolerance_rad
        return lin_tol_ok and ang_tol_ok and self.base_pose_received
