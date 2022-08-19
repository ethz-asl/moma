"""Define ROS bindings for mobile manipulation task."""

from typing import Any, Tuple
import rospy

import numpy as np
from numpy import linalg as LA

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest, Trigger
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates

# Action lib stuff
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mobile_manip_demo.msg import (
    GraspAction,
    GraspGoal,
    DropAction,
    DropGoal,
    RechargeAction,
    RechargeGoal,
)

"""
Actions return statuses:

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


# CONDITIONS
class RobotAtPose:
    """Check that the robot is at the target position."""

    def __init__(self, robot_name: str, namespace: str = "") -> None:
        """Initialize ROS nodes."""
        # TODO: rethink namespacing
        rospy.Subscriber(
            namespace + "amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb
        )
        rospy.loginfo(str("Subscribing to " + namespace + "amcl_pose ..."))
        self.current_pos = None
        # robot pose ground truth
        self.robot_name = robot_name
        self.ground_truth = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.robot_callback)
        rospy.loginfo(str("Subscribing to /gazebo/model_states ..."))

    def amcl_pose_cb(self, msg) -> None:
        self.current_pos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )

    def robot_callback(self, msg):
        if self.robot_name in msg.name:
            # the index may vary (don't know why)
            idx = 0
            for idx in range(len(msg.name)):
                if msg.name[idx] == self.robot_name:
                    break
                else:
                    continue
            self.ground_truth = np.array(
                [
                    msg.pose[idx].position.x,
                    msg.pose[idx].position.y,
                    msg.pose[idx].position.z,
                ]
            )
        else:
            self.ground_truth = np.array([100, 100, 0])

    def at_pose(
        self, target_pose: np.ndarray, tolerance: float, ground_truth: bool = False
    ) -> bool:
        """Check if the current pose is within tolerance from the target pose."""
        # Use e.g. [x,y] if the target is 2D
        reference = self.current_pos[: target_pose.size + 1]
        if ground_truth:
            reference = self.ground_truth[: target_pose.size + 1]
        distance = LA.norm(reference - target_pose)
        # TODO: check orientation?
        if distance < tolerance:
            return True
        return False


# TODO: check if this can be integrated in the relative one for the robot
class ObjectAtPose:
    """Check that the object is at the target position."""

    def __init__(self, target_object: str):
        """Initialize ROS nodes."""
        # object pose ground truth
        rospy.Subscriber("gazebo/model_states", ModelStates, self.obj_callback)
        self.target_obj = target_object
        self.current_pos = None
        self.ground_truth = None

    def obj_callback(self, msg):
        if self.target_obj in msg.name:
            # the index may vary (don't know why)
            idx = 0
            for idx in range(len(msg.name)):
                if msg.name[idx] == self.target_obj:
                    break
                else:
                    continue
            self.ground_truth = np.array(
                [
                    msg.pose[idx].position.x,
                    msg.pose[idx].position.y,
                    msg.pose[idx].position.z,
                ]
            )
        else:
            self.ground_truth = np.array([100, 100, 0])

    def at_pose(
        self, target_pose: np.ndarray, tolerance: float, ground_truth: bool = False
    ) -> bool:
        """Check if the current pose is within tolerance from the target pose."""
        # Use e.g. [x,y] if the target is 2D
        reference = self.current_pos[: target_pose.size + 1]
        if ground_truth:
            reference = self.ground_truth[: target_pose.size + 1]
        distance = LA.norm(reference - target_pose)
        # TODO: check orientation?
        if distance < tolerance:
            return True
        return False


class InHand:
    """Check if the object is hold in hand."""

    def __init__(self):
        """Initialize ROS nodes."""
        # TODO: check the robot state and the gripper state:
        # InHand = True if the gripper is closed but not fully

    def in_hand(self, target_object: str) -> bool:
        return True


# ACTIONS
class Move:
    """Low level implementation of a Navigation skill."""

    def __init__(self, namespace: str = "") -> None:
        """Initialize ROS nodes."""
        self.move_client = SimpleActionClient(namespace + "move_base", MoveBaseAction)
        rospy.loginfo(str("Connecting to " + namespace + "move_base ..."))
        self.move_client.wait_for_server()

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
        """Get result from navigation."""
        wait = self.move_client.wait_for_result()
        if not wait:
            rospy.logerr("Move Action server not available!")
            return -1
        else:
            # Result of executing the action
            return self.move_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        self.move_client.cancel_goal()


class Recharge(Move):
    """Low level implementation of a Recharge skill."""

    def __init__(self, namespace: str = "") -> None:
        """Initialize ROS nodes."""
        super().__init__(namespace)
        self.recharge_cli = SimpleActionClient("/recharge", RechargeAction)
        rospy.loginfo(str("Connecting to /recharge ..."))
        self.recharge_cli.wait_for_server()

    def initialize_recharge(self, recharge_pose: Pose) -> None:
        """Move the robot to the recharge station and recharge the robot."""
        super().initialize_navigation(recharge_pose, "map")
        rospy.loginfo(str("Initializing Recharge skill"))

    def get_recharge_status(self) -> int:
        """Get result from recharging."""
        state = super().get_navigation_status()
        if state == 3:
            # Navigation successful, then we can recharge
            goal_ = RechargeGoal()
            self.recharge_cli.send_goal(goal_)
            wait = self.recharge_cli.wait_for_result()
            if not wait:
                rospy.logerr("Recharge Action server not available!")
                return -1
            else:
                # Result of executing the action
                return self.recharge_cli.get_state()
        else:
            return state

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        super().cancel_goal()
        self.recharge_cli.cancel_goal()


class Pick:
    """Low level implementation of a Picking skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.pick_client = SimpleActionClient("/hl_grasp", GraspAction)
        rospy.loginfo(str("Connecting to /hl_grasp ..."))
        self.pick_client.wait_for_server()

    def initialize_pick(
        self,
        goal_pose: PoseStamped = PoseStamped(),
        goal_ID: int = -1,
    ) -> None:
        """
        Pick an item located in the target pose.

        Args:
        ----
            - goal_pose: if desired, set directly the goal to send.
            - goal_ID: a string ID for the item to pick.

        """
        # command
        goal_ = GraspGoal()
        goal_.target_object_pose = goal_pose
        goal_.goal_id = goal_ID

        # send the goal
        rospy.loginfo(
            f"Sending pick goal:\n--ID {goal_ID},\n--target: {goal_pose.pose},\n--reference frame: {goal_pose.header.frame_id}"
        )
        self.pick_client.send_goal(goal_)

    def get_pick_status(self) -> int:
        """Get result from pick."""
        # Block until you get result
        wait = self.pick_client.wait_for_result()
        if not wait:
            rospy.logerr("Pick Action server not available!")
            rospy.signal_shutdown("Pick Action server not available!")
            return -1
        else:
            # Result of executing the action
            return self.pick_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current pick goal."""
        self.pick_client.cancel_goal()


class Place:
    """Low level implementation of a Placing skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.place_client = SimpleActionClient("/hl_drop", DropAction)
        rospy.loginfo(str("Connecting to /hl_drop ..."))
        self.place_client.wait_for_server()

    def initialize_place(
        self,
        goal_pose: PoseStamped = PoseStamped(),
        goal_ID: int = -1,
    ) -> None:
        """
        Place an item in the target pose.

        Args:
        ----
            - goal_pose: if desired, set directly the goal to send.
            - goal_ID: a string ID for the picked item.

        """
        # command
        goal_ = DropGoal()
        goal_.target_object_pose = goal_pose
        goal_.goal_id = goal_ID

        # send the goal
        rospy.loginfo(
            f"Sending place goal:\n--ID {goal_ID},\n--target: {goal_pose.pose},\n--reference frame: {goal_pose.header.frame_id}"
        )
        self.place_client.send_goal(goal_)

    def get_place_status(self) -> int:
        """Get result from place."""
        wait = self.place_client.wait_for_result(rospy.Duration(10))
        if not wait:
            rospy.logerr("Place Action server not available!")
            rospy.signal_shutdown("Place Action server not available!")
            return -1
        else:
            # Result of executing the action
            return self.place_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current pick goal."""
        self.place_client.cancel_goal()


# ROBOT STATE
class State:
    """Represent part of the robot's internal state that cannot be measured."""

    def __init__(self):
        self.__holding = ""

    @property
    def holding(self):
        return self.__holding

    @holding.setter
    def holding(self, value):
        self.__holding = value
