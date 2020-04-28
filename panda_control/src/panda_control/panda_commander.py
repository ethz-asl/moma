import rospy

import actionlib
from franka_gripper.msg import *
import moveit_commander
from moveit_commander.conversions import list_to_pose


class PandaCommander(object):
    """
    TODO(mbreyer): write docstrings
    """

    def __init__(self, group_name="panda_arm"):
        # Connect to Panda MoveGroup
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Setup action clients to command the gripper
        self.home_client = actionlib.SimpleActionClient(
            "franka_gripper/homing", HomingAction
        )
        self.home_client.wait_for_server()

        self.move_client = actionlib.SimpleActionClient(
            "franka_gripper/move", MoveAction
        )
        self.move_client.wait_for_server()

        self.grasp_client = actionlib.SimpleActionClient(
            "franka_gripper/grasp", GraspAction
        )
        self.grasp_client.wait_for_server()
        rospy.loginfo("Connected to franka_gripper action servers")

    def goto_joint_target(
        self, joints, max_velocity_scaling=1.0, max_acceleration_scaling=1.0
    ):
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.move_group.set_joint_value_target(joints)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        return success

    def goto_pose_target(
        self, pose, max_velocity_scaling=1.0, max_acceleration_scaling=1.0
    ):
        pose_msg = list_to_pose(pose) if type(pose) is list else pose
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.move_group.set_pose_target(pose_msg)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def follow_cartesian_waypoints(self, poses):
        assert type(poses) == list

        waypoints = []
        for pose in poses:
            pose_msg = list_to_pose(pose) if type(pose) is list else pose
            waypoints.append(pose_msg)

        plan, fraction = self.move_group.compute_cartesian_path(
            waypoints=waypoints, eef_step=0.005, jump_threshold=0.0
        )
        if fraction != 1.0:
            raise ValueError("Unable to plan entire path!")

        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def home_gripper(self):
        self.home_client.send_goal(HomingGoal())
        return self.home_client.wait_for_result()

    def move_gripper(self, width, speed=0.1):
        goal = MoveGoal(width, speed)
        self.move_client.send_goal(goal)
        return self.move_client.wait_for_result()

    def _grasp(self, width, epsilon_inner=0.1, epsilon_outer=0.1, speed=0.1, force=1):
        grasp_epsilon = GraspEpsilon(epsilon_inner, epsilon_outer)
        goal = GraspGoal(width, grasp_epsilon, speed, force)
        self.grasp_client.send_goal(goal)
        return self.grasp_client.wait_for_result()

    def grasp(self):
        # TODO unify this with release. Does it make sense to use two different interfaces?
        self._grasp(0.05)

    def release(self):
        self.move_gripper(width=0.1)

    def check_object_grasped(self):
        # raise NotImplementedError
        # TODO need to implement this
        return True
