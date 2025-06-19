import moveit_commander
import numpy as np

from moma_utils.ros.conversions import *
from moma_utils.spatial import Transform

from geometry_msgs.msg import Pose, PoseStamped
import copy


class MoveItClient:
    def __init__(self, planning_group):
        self.planning_group = planning_group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)

    def goto(self, target, velocity_scaling=0.2, acceleration_scaling=0.2):
        plan = self.plan(target, velocity_scaling, acceleration_scaling)
        success = self.execute(plan)
        return success

    def plan(self, target, velocity_scaling=0.2, acceleration_scaling=0.2):
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)

        if isinstance(target, Transform):
            self.move_group.set_pose_target(to_pose_msg(target))
        elif isinstance(target, (list, np.ndarray)):
            self.move_group.set_joint_value_target(target)
        elif isinstance(target, str):
            self.move_group.set_named_target(target)
        else:
            raise ValueError

        plan = self.move_group.plan()
        if type(plan) is tuple:
            plan = plan[1]

        return plan

    def gotoL(self, target, velocity_scaling=0.1, acceleration_scaling=0.1):
        # this now fails, moveit changed the API
        # self.go_to_pose_goal_cartesian(pose_goal=target)
        plan = self.planL(target, velocity_scaling, acceleration_scaling)
        success = self.execute(plan)
        return success

    def planL(self, target, velocity_scaling=0.1, acceleration_scaling=0.1):
        # this now fails, moveit changed the API
        # plan = self.plan_cartesian_path()
        pose = to_pose_msg(target)
        waypoints = [pose]
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
        plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan

    def go_to_pose_goal_cartesian(
        self,
        pose_goal,
        vel_scaling = 0.1,
        acc_scaling = 0.1,
    ):
        if isinstance(pose_goal, Transform):
            self.move_group.set_pose_target(to_pose_msg(pose_goal))
        elif isinstance(pose_goal, Pose):
            self.move_group.set_pose_target(pose_goal)
        else:
            rospy.logerr(
                f"Pose goal must be of type Transform or Pose and is {type(pose_goal)}"
            )
            raise ValueError

        # plan = self.plan_cartesian_path(
        #     vel_scaling=vel_scaling, acc_scaling=acc_scaling
        # )
        # success = self.execute(plan)

        # return success

    def plan_cartesian_path(
        self, scale = 1.0, vel_scaling =0.1, acc_scaling = 0.1
    ) :
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

    def execute(self, plan):
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
