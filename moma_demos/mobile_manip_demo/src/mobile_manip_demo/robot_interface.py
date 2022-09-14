"""Define ROS bindings for mobile manipulation task."""

from copy import copy
from math import atan2, cos, sin
from typing import Any, List, Tuple

import rospy
import tf2_ros

import numpy as np
from numpy import linalg as LA

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from std_msgs.msg import Int32

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
from moma_utils.ros.panda import PandaGripperClient


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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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

    def compute_tf(self, target_frame: str, reference_frame: str):
        done = False
        attempts = 0
        while not done and attempts < 20:
            try:
                msg = self.tf_buffer.lookup_transform(
                    reference_frame,
                    target_frame,
                    rospy.Time(),
                    rospy.Duration(5),
                )
                done = True
            except Exception:
                attempts += 1
                rospy.logerr("Could not get transform, retrying...")
                rospy.Rate(1).sleep()

        if done:
            return msg
        else:
            return None


class RobotAtPose(MarkerPose):
    """Check that the robot is at the target position."""

    def __init__(self, robot_name: str) -> None:
        """Initialize ROS nodes."""
        super().__init__()
        rospy.Subscriber(
            "/mobile_base/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb
        )
        rospy.loginfo("Subscribing to /mobile_base/amcl_pose ...")
        self.amcl_pose = None
        # robot pose ground truth
        self.robot_name = robot_name
        self.gazebo_pose = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)
        rospy.loginfo("Subscribing to /gazebo/model_states ...")

    def amcl_pose_cb(self, msg) -> None:
        self.amcl_pose = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

    def gazebo_callback(self, msg):
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
                    msg.pose[idx].orientation.x,
                    msg.pose[idx].orientation.y,
                    msg.pose[idx].orientation.z,
                    msg.pose[idx].orientation.w,
                ]
            )
        else:
            self.gazebo_pose = np.array([100, 100, 0, 0, 0, 0, 1])

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
            target_pose = env.get_closest_robot_target(marker_pose)
            # target_pose = marker_pose
            # rospy.logwarn(f"Target pose is {target_pose}")
        target_pose_xy = target_pose[:2]
        target_pose_th = env.angle_from_quaternion(target_pose[3:], "yaw")

        if ground_truth or self.amcl_pose is None:
            current_pose_xy = self.gazebo_pose[:2]
            current_pose_th = env.angle_from_quaternion(self.gazebo_pose[3:], "yaw")
        else:
            current_pose_xy = self.amcl_pose[:2]
            current_pose_th = env.angle_from_quaternion(self.amcl_pose[3:], "yaw")
        # distance that the robot covers when controlled in velocity
        distance_xy = round(LA.norm(current_pose_xy - target_pose_xy) - 0.5, 3)
        delta_th = abs(
            atan2(
                sin(target_pose_th - current_pose_th),
                cos(target_pose_th - current_pose_th),
            )
        )
        distance_th = round(delta_th, 3)
        rospy.logwarn(
            f"Robot at distance {abs(distance_xy)}[m] and {distance_th}[rad] from target"
        )
        if abs(distance_xy) < tolerance and distance_th < 0.157:
            return True
        return False


class ObjectAtPose(MarkerPose):
    """Check that the object is at the target position."""

    def __init__(self, object_id: int, model_type: str = "cubes"):
        """Initialize ROS nodes."""
        # object pose ground truth
        super().__init__()
        rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_cb)
        self.gazebo_pose = None
        self.object_id = object_id
        self.target_obj, _ = env.get_item_by_marker(object_id, model_type)

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
        tolerance: np.ndarray,
    ) -> bool:
        """Check if the current pose is within tolerance from the target pose."""
        # Get target pose from marker
        rospy.sleep(3)
        current_pose = self.get_pose(self.object_id, np.ndarray)[:3]

        control_distance = abs(current_pose[:3] - self.gazebo_pose[:3])
        # rospy.logwarn(f"Distance marker-gazebo is {control_distance}")

        a, b, c = [control_distance[x] >= tolerance[x] for x in range(len(tolerance))]
        if (a or b or c) and not np.all(
            np.equal(self.gazebo_pose, np.array([100, 100, 0]))
        ):
            # this means that the marker is not updated correctly
            # then use the ground truth
            current_pose = self.gazebo_pose[:3]
            # rospy.logerr("Using control")

        distance = np.round(abs(current_pose[:3] - target_pose[:3]), 3)
        rospy.logwarn(f"Object is at {distance} from target")
        # TODO: check orientation?
        if np.all(distance <= tolerance):
            return True
        return False


class InHand:
    """Check if the object is hold in hand."""

    def __init__(self):
        """Initialize ROS nodes."""
        self.gripper = PandaGripperClient()

    def has_velocity(self) -> bool:
        # rospy.logerr(f"Velocity: {self.gripper.velocity()}")
        return True if self.gripper.velocity() > 0.5 else False

    def is_closed(self) -> bool:
        # rospy.logerr(f"Position: {self.gripper.read()}")
        return self.gripper.read() > 0.01 and self.gripper.read() < 0.07

    def has_effort(self) -> bool:
        # rospy.logerr(f"Effort: {self.gripper.effort()}")
        return True if self.gripper.effort() > 2 else False

    def in_hand(self) -> bool:
        if self.is_closed() and self.has_effort():  # and self.has_velocity():
            rospy.loginfo("Object grasped successfully")
            return True
        else:
            rospy.logwarn("Nothing detected in gripper")
            return False


class BatteryLv:
    """Check the battery Lv."""

    def __init__(self):
        """Initialize ROS nodes."""
        self.max_lv = rospy.get_param("moma_demo/battery_lv")
        self.current_lv = copy(self.max_lv)
        self.battery_sub = rospy.Subscriber("/battery_level", Int32, self.battery_cb)

    def battery_cb(self, msg):
        self.current_lv = msg.data if msg.data is not None else self.current_lv

    def battery_lv(self, relation: str, value: float) -> bool:
        current_rate = self.current_lv * 100 / self.max_lv
        if relation == "lower":
            out = True if current_rate < value else False
            # msg = f"Battery below {value}%" if out else f"Battery above {value}%"
        else:
            out = True if current_rate > value else False
            # msg = f"Battery above {value}%" if out else f"Battery below {value}%"

        # rospy.logwarn(msg)
        rospy.logwarn(f"Battery lv: {self.current_lv}")
        return out


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

    def found(self, IDs: List[int]) -> None:
        """Chack that n_IDs have been identified."""
        return all(x in list(self.marker_dict.keys()) for x in IDs)


# ACTIONS
class MoveArm:
    """Utility skill to easily move the arm from a configuration to another."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.moveit_client = MoveItClient("panda_arm")

    def initialize_arm(self, configuration: str) -> None:
        """Move the arm to the target configuration."""
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
        super().__init__()
        self.approach = True

        self.move_client = SimpleActionClient("/mobile_base/move_base", MoveBaseAction)
        rospy.loginfo("Connecting to /mobile_base/move_base ...")
        self.move_client.wait_for_server()

        self.vel_control = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel_control_sent = False

    def initialize_navigation(
        self,
        goal_pose: Pose or List[float] or np.ndarray = None,
        ref_frame: str = "map",
        goal_ID: int = None,
        approach=True,
    ) -> None:
        """
        Move the robot to the target pose.

        Args:
        ----
            - goal_pose: if desired, set directly the goal to send.
            - ref_frame: reference frame for the goal.
            - goal_ID: an int ID for the goal. If given, also goal_register must be provided.
            - approach: if True, send a velocity command to cover .5m

        """
        # rospy.logwarn(f"Got input: {goal_pose} and {goal_ID}.")
        self.approach = approach
        arm_client = MoveArm()
        arm_client.initialize_arm("detection")
        # Control the robot in velocity to make it go out the obstacles
        self.__velocity_control(velocity=-0.1)
        if goal_ID is not None:
            marker_pose = self.get_pose(goal_ID, np.ndarray)
            goal_pose = env.get_closest_robot_target(marker_pose)

        if goal_pose is None:
            goal_pose = Pose()
        elif type(goal_pose) != Pose:
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
        # rospy.logwarn(f"Sending goal:\n {goal_}")
        self.move_client.send_goal(goal_)

    def get_navigation_status(self) -> int:
        """Get result from navigation."""
        if self.move_client.get_state() == 3:
            if self.approach:
                rospy.logwarn("Navigation successful, we can move 50cm further")
                self.__velocity_control(velocity=0.1)
                # wait for the movement to end
                rospy.Rate(0.2).sleep()
            return 3

        else:
            return self.move_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        self.move_client.cancel_goal()

    def __velocity_control(self, velocity: float) -> None:
        """Control the robot on velocity, use carefully."""
        # Move velocity[m/s] * 5[s] further
        msg = Twist()
        msg.linear.x = velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(5):
            self.vel_control.publish(msg)
            rospy.Rate(10).sleep()
        rospy.logwarn("Forward movement ended")


class Recharge(Move):
    """Low level implementation of a Recharge skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        super().__init__()
        self.recharge_pose = rospy.get_param("moma_demo/battery_station")
        self.recharge_cli = SimpleActionClient("/recharge", RechargeAction)
        rospy.loginfo("Connecting to /recharge ...")
        self.recharge_cli.wait_for_server()

    def initialize_recharge(self) -> None:
        """Move the robot to the recharge station and recharge the robot."""
        super().initialize_navigation(
            goal_pose=self.recharge_pose, ref_frame="map", approach=False
        )
        rospy.loginfo("Initializing Recharge skill")

    def get_recharge_status(self) -> int:
        """Get result from recharging."""
        state = super().get_navigation_status()
        if state == 3:
            # Navigation successful, then we can recharge
            rospy.logerr("Move-To charge pose successful!")
            goal_ = RechargeGoal()
            # rospy.loginfo("Sending recharge request.")
            self.recharge_cli.send_goal(goal_)
            return 3 if self.recharge_cli.get_state() == 0 else 4
        else:
            return state

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        super().cancel_goal()
        self.recharge_cli.cancel_goal()


class Dock(Move):
    """Low level implementation of a Docking skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        super().__init__()
        self.dock_pose = rospy.get_param("moma_demo/inspection_station")

    def initialize_docking(self) -> None:
        """Move the robot to the docking station."""
        super().initialize_navigation(
            goal_pose=self.dock_pose, ref_frame="map", approach=True
        )
        rospy.loginfo("Initializing Docking skill")

    def get_docking_status(self) -> int:
        """Get result from recharging."""
        state = super().get_navigation_status()
        return state

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        super().cancel_goal()


class Search(Move):
    """Low level implementation of a search skill."""

    def __init__(self, targets: List[List[float]]) -> None:
        """Initialize ROS nodes."""
        super().__init__()
        self.initial_targets = targets
        self.current_targets = None

    def initialize_search(self) -> None:
        """Move the robot to the target poses."""
        rospy.loginfo("Initializing Search skill")
        self.current_targets = copy(self.initial_targets, approach=False)
        for target in self.current_targets:
            # command
            self.initialize_navigation(goal_pose=target)
            self.targets.pop(0)
            self.move_client.wait_for_result()

    def get_search_status(self) -> int:
        """Get result from searching."""
        state = super().get_navigation_status()
        if state == 3:
            if len(self.current_targets) == 0:
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


class Pick:
    """Low level implementation of a Picking skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.pick_client = SimpleActionClient("/hl_grasp", GraspAction)
        rospy.loginfo("Connecting to /hl_grasp ...")
        self.pick_client.wait_for_server()

    def initialize_pick(
        self,
        goal_pose: Pose or List[float] or np.ndarray = None,
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
        # rospy.logwarn(f"Got input: {goal_pose} and {goal_ID}.")
        goal_ = GraspGoal()
        goal_.target_object_pose = PoseStamped()
        goal_.target_object_pose.header.frame_id = "panda_link0"

        if goal_pose is None:
            goal_pose = Pose()
        elif type(goal_pose) != Pose:
            new_goal_pose = Pose()
            new_goal_pose.position.x = goal_pose[0]
            new_goal_pose.position.y = goal_pose[1]
            new_goal_pose.position.z = goal_pose[2]

            goal_pose = copy(new_goal_pose)

        goal_.target_object_pose.pose = goal_pose
        goal_.goal_id = goal_ID

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
        rospy.loginfo("Connecting to /hl_drop ...")
        self.place_client.wait_for_server()

    def initialize_place(
        self,
        goal_pose: Pose or List[float] or np.ndarray = Pose(),
        goal_ID: int = -1,
    ) -> None:
        """
        Place an item in the target pose in the manipulator frame.

        Args:
        ----
            - goal_pose: if desired, set directly the goal to send.
            - goal_ID: a string ID for the picked item.

        """
        # command
        # rospy.logwarn(f"Got input: {goal_pose} and {goal_ID}.")
        goal_ = DropGoal()
        goal_.target_object_pose = PoseStamped()
        goal_.target_object_pose.header.frame_id = "panda_link0"

        if type(goal_pose) != Pose:
            new_goal_pose = Pose()
            new_goal_pose.position.x = np.random.uniform(
                goal_pose[0] + 0.1, goal_pose[0] - 0.1
            )
            new_goal_pose.position.y = np.random.uniform(
                goal_pose[1] + 0.2, goal_pose[1] - 0.2
            )
            new_goal_pose.position.z = goal_pose[2] if goal_pose[2] >= 0.25 else 0.25

            # the gripper is oriented 'downward'
            new_goal_pose.orientation.x = 1.0

            goal_pose = copy(new_goal_pose)

        goal_.target_object_pose.pose = goal_pose
        goal_.goal_id = goal_ID

        self.place_client.send_goal(goal_)

    def get_place_status(self) -> int:
        """Get result from place."""
        return self.place_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current pick goal."""
        self.place_client.cancel_goal()
