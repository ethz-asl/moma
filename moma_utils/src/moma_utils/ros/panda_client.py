#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function, annotations
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import actionlib
import numpy as np
import scipy as sc
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# from scipy.spatial.transform import Rotation
from moma_utils.ros import conversions
from moma_utils.transform import Transform
from franka_gripper.msg import (
    GraspAction,
    GraspGoal,
    GraspEpsilon,
    StopAction,
    StopGoal,
    MoveAction,
    MoveGoal,
    HomingAction,
    HomingGoal,
)
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal, FrankaState

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from moveit_commander.conversions import pose_to_list


def all_close(
    goal: list | PoseStamped | Pose,
    actual: list | PoseStamped | Pose,
    tolerance: float = 0.01,
) -> bool:
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class PandaArmClient(object):
    """MoveIt Arm client for panda manipulator"""

    def __init__(self, group_name: str = "panda_arm") -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        # a node can't start another node!
        #rospy.init_node("move_group_client", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = move_group.get_planning_frame()
        rospy.loginfo("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        rospy.loginfo(f"============ End effector link: {str(eef_link)}")

        group_names = robot.get_group_names()
        rospy.loginfo(f"============ Available Planning Groups: {robot.get_group_names()}")

        named_targets = move_group.get_named_targets()
        rospy.loginfo(f"============ Named targets {str(named_targets)}")

        rospy.loginfo(f"{group_name} is ready for MoveIt commands")
        self._init_state_publishers()
        self._init_recovery()

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.named_targets = named_targets
        self.status: int = 0

    def _init_state_callbacks(self):
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)
        rospy.wait_for_message("joint_states", JointState)
        rospy.Subscriber(
            "franka_state_controller/franka_states", FrankaState, self._robot_state_cb
        )

    def _init_state_publishers(self):
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )

    def _init_recovery(self):
        self.has_error = False
        self.recovery_client = actionlib.SimpleActionClient(
            "franka_control/error_recovery", ErrorRecoveryAction
        )

    def _joint_state_cb(self, msg):
        self._joint_state_msg = msg

    def _robot_state_cb(self, msg):
        if msg.robot_mode == 4:
            self.has_error = True
        else:
            self.has_error = False

    def _check_robot_state(self):
        state = rospy.wait_for_message(
            "franka_state_controller/franka_states", FrankaState
        )
        if state.robot_mode == 4:
            return False
        else:
            return True

    def get_state(self):
        q = np.asarray(self._joint_state_msg.position[:7])
        dq = np.asarray(self._joint_state_msg.velocity[:7])
        return q, dq

    def recover(self):
        msg = ErrorRecoveryActionGoal()
        self.recovery_client.send_goal(msg)
        self.recovery_client.wait_for_result(timeout=rospy.Duration(10.0))
        if not self._check_robot_state():
            rospy.sleep(rospy.Duration(3.0))
            if not self._check_robot_state():
                rospy.logerr("Arm error recovery failed")
                return
        self.has_error = False
        rospy.loginfo("Arm error recovered")

    def goto(
        self,
        target: Transform | list | np.ndarray | str,
        vel_scale: float = 0.2,
        acc_scale: float = 0.2,
    ) -> bool:
        plan = self.plan(target, vel_scale, acc_scale)
        return self.execute(plan)

    def plan(
        self,
        target: Transform | list | np.ndarray | str,
        vel_scaling: float = 0.2,
        acc_scaling: float = 0.2,
    ) -> RobotTrajectory:
        self.move_group.set_max_velocity_scaling_factor(vel_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acc_scaling)

        if isinstance(target, Transform):
            self.move_group.set_pose_target(conversions.to_pose_msg(target))
        elif isinstance(target, (list, np.ndarray)):
            self.move_group.set_joint_value_target(target)
        elif isinstance(target, str):
            self.move_group.set_named_target(target)
        else:
            rospy.logerr(f"Type of target goal, {type(target)}, is not valid")
            raise ValueError

        plan = self.move_group.plan()
        if type(plan) is tuple:
            plan = plan[1]

        return plan

    def execute(self, plan: RobotTrajectory) -> bool:
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def go_to_named_target(self, target: str) -> bool:
        """go to named target"""
        return self.go_to_joint_goal(self.move_group.get_named_target_values(target))

    def go_to_floor(self) -> bool:
        return self.go_to_named_target("floor")

    def go_to_home(self) -> bool:
        return self.go_to_named_target("home")

    def go_to_safe(self) -> bool:
        return self.go_to_named_target("safe")

    def go_to_joint_goal(self, joint_goal: dict) -> bool:
        """
        Planning to a Joint Goal
        """
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints)

    def get_pose(self, position: np.array, orientation: np.array) -> Pose:
        pose_goal = Pose()
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]

        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        return pose_goal

    def go_to_pose_goal(self, pose_goal: Pose | Transform) -> bool:
        """
        Planning to a Pose Goal
        Plan a motion for this group to a desired pose for the
        end-effector:
        """
        if isinstance(pose_goal, Transform):
            self.move_group.set_pose_target(conversions.to_pose_msg(pose_goal))
        elif isinstance(pose_goal, Pose):
            self.move_group.set_pose_target(pose_goal)
        else:
            rospy.logerr(
                f"Pose goal must be of type Transform or Pose and is {type(pose_goal)}"
            )
            raise ValueError

        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose)

    def go_to_pose_goal_cartesian(
        self,
        pose_goal: Pose | Transform,
        vel_scaling: float = 0.1,
        acc_scaling: float = 0.1,
    ) -> bool:
        if isinstance(pose_goal, Transform):
            self.move_group.set_pose_target(conversions.to_pose_msg(pose_goal))
        elif isinstance(pose_goal, Pose):
            self.move_group.set_pose_target(pose_goal)
        else:
            rospy.logerr(
                f"Pose goal must be of type Transform or Pose and is {type(pose_goal)}"
            )
            raise ValueError

        plan = self.plan_cartesian_path(
            vel_scaling=vel_scaling, acc_scaling=acc_scaling
        )
        success = self.execute(plan)

        return success

    def plan_cartesian_path(
        self, scale: float = 1.0, vel_scaling: float = 0.1, acc_scaling: float = 0.1
    ) -> tuple[RobotTrajectory, float]:
        """"""
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, _) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        state = self.robot.get_current_state()

        return self.move_group.retime_trajectory(
            state,
            plan,
            velocity_scaling_factor=vel_scaling,
            acceleration_scaling_factor=acc_scaling,
            algorithm="time_optimal_trajectory_generation",
        )

    def display_trajectory(self, plan: RobotTrajectory) -> None:
        """"""
        self.display_trajectory = DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(self.display_trajectory)

    def execute_plan(self, plan: RobotTrajectory) -> None:
        """
        Executing a Plan
        """
        self.move_group.execute(plan, wait=True)

    def cancel_goal(self) -> None:
        """Cancel the current goal"""
        rospy.loginfo("Cancelling MoveIt goal")
        self.move_group.stop()


class PandaGripperClient(object):
    def __init__(self, ns : str = "panda/franka_gripper/"):
        self._init_state_callback()
        self._init_action_clients(ns = ns)
        rospy.loginfo("Panda gripper ready")

    def home(self):
        msg = HomingGoal()
        self.homing_client.send_goal(msg)
        self.homing_client.wait_for_result(rospy.Duration.from_sec(20.0))
        rospy.loginfo("Panda gripper finished homing")

    def move(self, width: float, speed: float = 0.1):
        msg = MoveGoal(width, speed)
        self.move_client.send_goal(msg)
        self.move_client.wait_for_result(rospy.Duration.from_sec(2.0))

    def grasp(
        self,
        width: float = 0.0,
        e_inner: float = 0.1,
        e_outer: float = 0.1,
        speed: float = 0.1,
        force: float = 5.0,
    ):
        rospy.loginfo("Closing gripper")
        msg = GraspGoal(width, GraspEpsilon(e_inner, e_outer), speed, force)
        self.grasp_client.send_goal(msg)
        self.grasp_client.wait_for_result(rospy.Duration(2.0))

    def release(self, width: float = 0.1):
        rospy.loginfo("Opening gripper")
        self.move(width)

    def stop(self):
        msg = StopGoal()
        self.stop_client.send_goal(msg)
        self.stop_client.wait_for_result(timeout=rospy.Duration(2.0))

    def read(self) -> float:
        return self._joint_state_msg.position[7] + self._joint_state_msg.position[8]

    def _init_state_callback(self):
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)

    def _joint_state_cb(self, msg : JointState):
        self._joint_state_msg = msg

    def _init_action_clients(self, ns : str = "franka_gripper/"):
        self.move_client = actionlib.SimpleActionClient(ns + "move", MoveAction)
        self.grasp_client = actionlib.SimpleActionClient(ns + "grasp", GraspAction)
        self.stop_client = actionlib.SimpleActionClient(ns + "stop", StopAction)
        self.homing_client = actionlib.SimpleActionClient(ns + "homing", HomingAction)
        rospy.loginfo(f"Waiting for {ns}")
        self.move_client.wait_for_server()
        self.grasp_client.wait_for_server()
        self.stop_client.wait_for_server()
        self.homing_client.wait_for_server()
        rospy.loginfo("Gripper connected")


def main():
    rospy.init_node("test_panda_client")

    arm = PandaArmClient()
    gripper = PandaGripperClient()

    # prep robot    
    arm.go_to_home()
    gripper.home()

    while True:
        try:
            print("")
            print("----------------------------------------------------------")
            print("Test MoveIt client")
            print("----------------------------------------------------------")
            print("Press 'h' to go to the arm home state")
            print("----------------------------------------------------------")
            print("Press 'o' to open grippers")
            print("----------------------------------------------------------")
            print("Press 'c' to close grippers")
            print("----------------------------------------------------------")
            print("Press 'j' to go to a joint state (preprogrammed)")
            print("----------------------------------------------------------")
            print("Press 'l' to go to a pose state (linear/cartesian)")
            print("----------------------------------------------------------")
            print("Press 'q' to stop sampling exit")

            print("")

            user_input = input("============ Choose an option: ")
            print("")

            if user_input == "h":
                arm.go_to_home()
                gripper.home()
            elif user_input == "o":
                gripper.release()
            elif user_input == "c":
                gripper.grasp()
            elif user_input == "j":
                arm.goto([0.605, -0.311, -0.163, -2.341, -0.077, 2.133, 1.248])
            elif user_input == "l":
                pose = Pose()
                pose = arm.get_pose([0.5, 0.0, 0.2], [0.707, 0.0, 0.707, 0.0])
                arm.go_to_pose_goal_cartesian(pose)
            elif user_input == "q":
                input("============ Ending program")
                break

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    main()
