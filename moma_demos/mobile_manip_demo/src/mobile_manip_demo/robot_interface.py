"""Define ROS bindings for mobile manipulation task."""

from typing import Tuple
import rospy

import numpy as np
from numpy import linalg as LA

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates

# Action lib stuff
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Custom actions
from mobile_manip_demo.grasp import GraspAction


# CONDITIONS
class RobotAtPose:
    """Check that the robot is at the target position."""

    def __init__(self, robot_name: str) -> None:
        """Initialize ROS nodes."""
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb)
        self.current_pos = None
        # robot pose ground truth
        self.robot_name = robot_name
        self.ground_truth = None
        rospy.Subscriber("gazebo/model_states", ModelStates, self.robot_callback)

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

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.move_client = SimpleActionClient("move_client", MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(100))

    def initialize_navigation(self, goal_ID: str) -> None:
        """Move the robot to the target pose."""
        target_goal, ref_frame = get_goal_from_ID(goal_ID)
        target_pose, target_orientation = target_goal
        # command
        goal_ = MoveBaseGoal()
        goal_.target_pose.header.frame_id = ref_frame
        goal_.target_pose.header.stamp = rospy.Time.now()
        goal_.target_pose.pose.position.x = target_pose[0]
        goal_.target_pose.pose.position.y = target_pose[1]
        goal_.target_pose.pose.position.z = target_pose[2]
        goal_.target_pose.pose.orientation.x = target_orientation[0]
        goal_.target_pose.pose.orientation.y = target_orientation[1]
        goal_.target_pose.pose.orientation.z = target_orientation[2]
        goal_.target_pose.pose.orientation.w = target_orientation[3]

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
        wait = self.move_client.wait_for_result(rospy.Duration(10))
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


class Pick:
    """Low level implementation of a Picking skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.pick_client = SimpleActionClient("pick_client", GraspAction)
        self.pick_client.wait_for_server(rospy.Duration(100))

    def initialize_pick(self, goal_ID: str) -> None:
        """Move the robot to the target pose."""
        # TODO: implement method

        # send the goal
        self.pick_client.send_goal(goal_ID)

    def get_pick_status(self) -> int:
        """Get result from pick."""
        wait = self.pick_client.wait_for_result(rospy.Duration(10))
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
        self.place_client = SimpleActionClient("place_client", PlaceAction)
        self.place_client.wait_for_server(rospy.Duration(100))

    def initialize_place(self, goal_ID: str) -> None:
        """Move the robot to the target pose."""
        # TODO: implement method

        # send the goal
        self.place_client.send_goal(goal_ID)

    def get_place_status(self) -> int:
        """Get result from place."""
        wait = self.place_client.wait_for_result(rospy.Duration(10))
        if not wait:
            rospy.logerr("Pick Action server not available!")
            rospy.signal_shutdown("Pick Action server not available!")
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
