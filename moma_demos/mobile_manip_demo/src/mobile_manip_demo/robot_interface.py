"""Define ROS bindings for mobile manipulation task."""

from copy import copy
from typing import Any, List, Tuple
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
from mobile_manip_demo import environment as env
from mobile_manip_demo.msg import (
    GraspAction,
    GraspGoal,
    DropAction,
    DropGoal,
    MarkerPoses,
    RechargeAction,
    RechargeGoal,
)
from moma_utils.ros.moveit import MoveItClient
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient


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
class MarkerPose:
    """Perception Skill to get the marker pose."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        rospy.Subscriber("/marker_poses", MarkerPoses, self.marker_cb)
        self.marker_dict = {}

    def marker_cb(self, msg) -> None:
        """Read the marker msg with IDs and last known poses."""
        for i, id in enumerate(msg.marker_ids):
            self.marker_dict[id] = msg.poses[i]

    def get_pose(
        self, item_ID: int, msg_type: Any = np.ndarray
    ) -> np.ndarray or list or Pose:
        marker_pose = self.marker_dict[item_ID]
        as_list = [
            marker_pose.position.x,
            marker_pose.position.y,
            marker_pose.position.z,
            marker_pose.orientation.x,
            marker_pose.orientation.y,
            marker_pose.orientation.z,
            marker_pose.orientation.w,
        ]
        if msg_type == np.ndarray:
            return np.array(as_list)
        elif msg_type == List:
            return as_list
        else:
            return marker_pose


class RobotAtPose(MarkerPose):
    """Check that the robot is at the target position."""

    def __init__(self, robot_name: str) -> None:
        """Initialize ROS nodes."""
        rospy.Subscriber(
            "/mobile_base/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb
        )
        rospy.loginfo(str("Subscribing to /mobile_base/amcl_pose ..."))
        self.amcl_pose = None
        # robot pose ground truth
        self.robot_name = robot_name
        self.gazebo_pose = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.robot_callback)
        rospy.loginfo(str("Subscribing to /gazebo/model_states ..."))

        super().__init__()

    def amcl_pose_cb(self, msg) -> None:
        self.amcl_pose = np.array(
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
            self.gazebo_pose = np.array(
                [
                    msg.pose[idx].position.x,
                    msg.pose[idx].position.y,
                    msg.pose[idx].position.z,
                ]
            )
        else:
            self.gazebo_pose = np.array([100, 100, 0])

    def at_pose(
        self,
        target_pose: np.ndarray or int,
        tolerance: float,
        ground_truth: bool = False,
    ) -> bool:
        """Check if the current pose is within tolerance from the target pose."""
        if type(target_pose) == int:
            # Get target pose from marker
            marker_pose = self.get_pose(target_pose, np.ndarray)
            target_pose = env.get_closest_robot_target(marker_pose)[:2]
        # 2D target
        current_pose = self.amcl_pose[:2]
        if ground_truth:
            current_pose = self.gazebo_pose[:2]
        distance = LA.norm(current_pose - target_pose)
        if distance < tolerance:
            return True
        return False


# TODO: check if this can be integrated in the relative one for the robot
class ObjectAtPose(MarkerPose):
    """Check that the object is at the target position."""

    def __init__(self, object_id: int, model_type: str = "cubes"):
        """Initialize ROS nodes."""
        # object pose ground truth
        rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_cb)
        self.gazebo_pose = None
        self.object_id = object_id
        self.target_obj = env.get_item_by_marker(object_id, model_type)

        super().__init__()

    def gazebo_cb(self, msg):
        if self.target_obj in msg.name:
            # the index may vary (don't know why)
            idx = 0
            for idx in range(len(msg.name)):
                if msg.name[idx] == self.target_obj:
                    break
                else:
                    continue
            self.gazebo_pose = np.array(
                [
                    msg.pose[idx].position.x,
                    msg.pose[idx].position.y,
                    msg.pose[idx].position.z,
                ]
            )
        else:
            self.gazebo_pose = np.array([100, 100, 0])

    def at_pose(
        self,
        target_pose: np.ndarray,
        tolerance: float,
        ground_truth: bool = False,
    ) -> bool:
        """Check if the current pose is within tolerance from the target pose."""
        # Get target pose from marker
        current_pose = self.get_pose(self.object_id, np.ndarray)[:3]
        if ground_truth:
            current_pose = self.gazebo_pose[:3]
        distance = LA.norm(current_pose - target_pose)
        # TODO: check orientation?
        if distance < tolerance:
            return True
        return False


class InHand:
    """Check if the object is hold in hand."""

    def __init__(self):
        """Initialize ROS nodes."""
        self.gripper = PandaGripperClient()

    def in_hand(self) -> bool:
        if self.gripper.read() > 0.002:
            rospy.loginfo("Object grasped successfully")
            return True
        else:
            rospy.logwarn("Nothing detected in gripper")
            return False


class Found:
    """Check that the search motion was successfull."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.marker_sub = rospy.Subscriber("/marker_poses", MarkerPoses, self.marker_cb)
        self.marker_dict = {}

    def marker_cb(self, msg) -> None:
        """Read the marker msg with IDs and last known poses."""
        for i, id in enumerate(msg.marker_ids):
            self.marker_dict[id] = msg.poses[i]

    def found(self, n_IDs: int = 3) -> None:
        """Chack that n_IDs have been identified."""
        if len(self.marker_dict.keys()) == n_IDs:
            return True
        return False


# ACTIONS
class MoveArm:
    """Utility skill to easily move the arm from a configuration to another."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.moveit_client = MoveItClient("panda_arm")

    def initialize_arm(self, configuration: str) -> None:
        """
        Move the arm to the target configuration.

        Args:
        ----
            - configuration: the target for the arm motion.

        """
        self.moveit_client.goto(configuration)

    def get_motion_status(self) -> int:
        """Get result from navigation."""
        return self.moveit_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current motion goal."""
        self.moveit_client.cancel_goal()


class Move(MarkerPose):
    """Low level implementation of a Navigation skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.move_client = SimpleActionClient("/mobile_base/move_base", MoveBaseAction)
        rospy.loginfo(str("Connecting to /mobile_base/move_base ..."))
        self.move_client.wait_for_server()

        self.vel_control = rospy.Publisher(
            "/mobile_base/ridgeback_velocity_controller/cmd_vel", Twist, queue_size=10
        )

        super().__init__()

    def initialize_navigation(
        self,
        goal_pose: Pose or List[float] or np.ndarray = None,
        ref_frame: str = "map",
        goal_ID: int = None,
        approach: bool = True,
    ) -> None:
        """
        Move the robot to the target pose.

        Args:
        ----
            - goal_pose: if desired, set directly the goal to send.
            - ref_frame: reference frame for the goal.
            - goal_ID: an int ID for the goal. If given, also goal_register must be provided.
            - approach: add a pre- and post-goal motion to the robot.

        """
        arm_client = MoveArm()
        arm_client.initialize_arm("detection")
        if goal_ID is not None:
            marker_pose = self.get_pose(goal_ID, np.ndarray)
            goal_pose = env.get_closest_robot_target(marker_pose)

        if type(goal_pose) != Pose():
            new_goal_pose = Pose()
            new_goal_pose.position.x = goal_pose[0]
            new_goal_pose.position.y = goal_pose[1]
            new_goal_pose.position.z = 0.0
            try:
                new_goal_pose.orientation.x = goal_pose[3]
                new_goal_pose.orientation.y = goal_pose[4]
                new_goal_pose.orientation.z = goal_pose[5]
                new_goal_pose.orientation.w = goal_pose[6]
            except:
                pass
            goal_pose = copy(new_goal_pose)

        # command
        goal_ = MoveBaseGoal()
        goal_.target_pose.header.frame_id = ref_frame
        goal_.target_pose.header.stamp = rospy.Time.now()
        goal_.target_pose.pose = goal_pose

        # send the goal
        # TODO: if approach send a vel command to the robot
        self.move_client.send_goal(goal_)

    def get_navigation_status(self) -> int:
        """Get result from navigation."""
        if self.move_client.get_state() == 3:
            # Success! we can move 50cm further
            msg = Twist()
            msg.linear.x = 0.1
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            start = rospy.Time.now()
            while rospy.Time.now() - start < rospy.Duration(5):
                self.vel_control.publish(msg)
                rospy.Rate(10).sleep()
        return self.move_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        self.move_client.cancel_goal()


class Recharge(Move):
    """Low level implementation of a Recharge skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        super().__init__()
        self.recharge_cli = SimpleActionClient("/recharge", RechargeAction)
        rospy.loginfo(str("Connecting to /recharge ..."))
        self.recharge_cli.wait_for_server()

    def initialize_recharge(self, recharge_pose: Pose) -> None:
        """Move the robot to the recharge station and recharge the robot."""
        super().initialize_navigation(recharge_pose, "map", approach=False)
        rospy.loginfo(str("Initializing Recharge skill"))

    def get_recharge_status(self) -> int:
        """Get result from recharging."""
        state = super().get_navigation_status()
        if state == 3:
            # Navigation successful, then we can recharge
            goal_ = RechargeGoal()
            self.recharge_cli.send_goal(goal_)
            return self.recharge_cli.get_state()
        else:
            return state

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        super().cancel_goal()
        self.recharge_cli.cancel_goal()


class Search(Move):
    """Low level implementation of a search skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        super().__init__()
        self.targets = None

    def initialize_search(self, targets: List[List[float]]) -> None:
        """Move the robot to the target poses."""
        self.targets = self.__make_poses(targets)
        targets_copy = copy(self.targets)
        rospy.loginfo(str("Initializing Search skill"))
        for target in targets_copy:
            # command
            self.initialize_navigation(goal_pose=target)
            self.targets.pop(0)
            self.move_client.wait_for_result()

    def get_search_status(self) -> int:
        """Get result from searching."""
        state = super().get_navigation_status()
        if state == 3:
            if len(self.targets) == 0:
                # then we are fully done
                self.cancel_goal()
                return state
            else:
                # still running
                return 1
        else:
            self.cancel_goal()
            return state

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        super().cancel_goal()
        self.targets = None

    def __make_poses(self, waypoints: List[List[float]]) -> List[Pose]:
        """Read the parameters and build a Pose msg."""
        pose_list = []
        for point in waypoints:
            msg = Pose()
            msg.position.x = point[0]
            msg.position.y = point[1]
            msg.position.z = point[2]
            msg.orientation.x = point[3]
            msg.orientation.y = point[4]
            msg.orientation.z = point[5]
            msg.orientation.w = point[6]
            pose_list.append(msg)

        return pose_list


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
        return self.place_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current pick goal."""
        self.place_client.cancel_goal()
